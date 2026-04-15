/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/audio/codec.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define I2S_CODEC_BUS DT_ALIAS(i2s_codec_tx)
#define CODEC_NODE    DT_NODELABEL(audio_codec)
#define CODEC_I2C_BUS  DT_BUS(CODEC_NODE)
#define CODEC_I2C_ADDR DT_REG_ADDR(CODEC_NODE)

/* WM8904 register map used for debug readback. */
#define WM8904_REG_POWER_MGMT_0      (0x0C)
#define WM8904_REG_POWER_MGMT_2      (0x0E)
#define WM8904_REG_POWER_MGMT_6      (0x12)
#define WM8904_REG_ANALOG_LEFT_IN_0  (0x2C)
#define WM8904_REG_ANALOG_RIGHT_IN_0 (0x2D)
#define WM8904_REG_ANALOG_LEFT_IN_1  (0x2E)
#define WM8904_REG_ANALOG_RIGHT_IN_1 (0x2F)
#define WM8904_REG_ANALOG_OUT12_ZC   (0x3D)
#define WM8904_REG_ANALOG_HP_0       (0x5A)

#define SAMPLE_FREQUENCY CONFIG_SAMPLE_FREQ
#define SAMPLE_BIT_WIDTH 16U
#define CHANNEL_COUNT    2U

#define BYTES_PER_SAMPLE sizeof(int16_t)
#define FRAMES_PER_BLOCK 480U
#define BLOCK_SIZE       (FRAMES_PER_BLOCK * CHANNEL_COUNT * BYTES_PER_SAMPLE)
#define BLOCK_COUNT      16U
#define TIMEOUT_MS       SYS_FOREVER_MS
#define REPORT_BLOCKS    100U
#define TX_PRIME_BLOCKS  3U /* Keep below the DW TX queue depth so startup cannot block. */
#define TONE_PERIOD_FRAMES 48U
#define TONE_AMPLITUDE      6000
#define PLAYBACK_BLOCKS     200U

static const uint8_t capture_input = 2U;

K_MEM_SLAB_DEFINE_STATIC(tx_mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

static int configure_codec(const struct device *codec_dev, audio_route_t route)
{
	struct audio_codec_cfg audio_cfg = {0};
	int ret;

	audio_cfg.dai_route = route;
	audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
	audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
	audio_cfg.dai_cfg.i2s.channels = CHANNEL_COUNT;
	audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
	audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_FREQUENCY;
	audio_cfg.dai_cfg.i2s.mem_slab = &tx_mem_slab;
	audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
	audio_cfg.dai_cfg.i2s.timeout = TIMEOUT_MS;

	ret = audio_codec_configure(codec_dev, &audio_cfg);
	if (ret < 0) {
		printk("Failed to configure codec route %d: %d\n", route, ret);
		return ret;
	}

	/* Keep the codec output path active for the selected route. */
	audio_codec_start_output(codec_dev);

	return 0;
}

static int select_capture_input(const struct device *codec_dev, uint8_t input)
{
	int ret;

	/*
	 * The upstream WM8904 driver routes input per channel, so program both
	 * stereo channels explicitly.
	 */
	ret = audio_codec_route_input(codec_dev, AUDIO_CHANNEL_FRONT_LEFT, input);
	if (ret < 0) {
		printk("Failed to route WM8904 left capture input IN%u: %d\n", input, ret);
		return ret;
	}

	ret = audio_codec_route_input(codec_dev, AUDIO_CHANNEL_FRONT_RIGHT, input);
	if (ret < 0) {
		printk("Failed to route WM8904 right capture input IN%u: %d\n", input, ret);
		return ret;
	}

	printk("WM8904 capture input set to IN%u\n", input);
	return 0;
}

static int configure_i2s_dir(const struct device *i2s_dev, enum i2s_dir dir,
			     struct k_mem_slab *mem_slab);

static int prime_tx_path(const struct device *i2s_dev);

static int16_t tone_sample(uint32_t phase)
{
	const uint32_t half_period = TONE_PERIOD_FRAMES / 2U;

	if (phase < half_period) {
		return (int16_t)(-TONE_AMPLITUDE +
				 (int32_t)(((int32_t)2 * TONE_AMPLITUDE * (int32_t)phase) / half_period));
	}

	phase -= half_period;
	return (int16_t)(TONE_AMPLITUDE -
			 (int32_t)(((int32_t)2 * TONE_AMPLITUDE * (int32_t)phase) / half_period));
}

static void fill_tone_block(int16_t *samples, size_t frame_count, uint32_t *phase)
{
	for (size_t frame = 0U; frame < frame_count; ++frame) {
		const int16_t sample = tone_sample(*phase);

		samples[(frame * CHANNEL_COUNT) + 0U] = sample;
		samples[(frame * CHANNEL_COUNT) + 1U] = sample;
		*phase = (*phase + 1U) % TONE_PERIOD_FRAMES;
	}
}

static int play_tone_phase(const struct device *i2s_dev)
{
	uint32_t tone_phase = 0U;
	int ret;

	ret = configure_i2s_dir(i2s_dev, I2S_DIR_TX, &tx_mem_slab);
	if (ret < 0) {
		return ret;
	}

	ret = prime_tx_path(i2s_dev);
	if (ret < 0) {
		return ret;
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("Failed to start I2S TX: %d\n", ret);
		return ret;
	}

	for (uint32_t block = 0U; block < PLAYBACK_BLOCKS; ++block) {
		void *tx_block = NULL;
		int16_t *samples;

		ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
		if (ret < 0) {
			printk("Failed to allocate tone block: %d\n", ret);
			break;
		}

		samples = (int16_t *)tx_block;
		fill_tone_block(samples, FRAMES_PER_BLOCK, &tone_phase);

		ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			k_mem_slab_free(&tx_mem_slab, tx_block);
			printk("Failed to queue tone block: %d\n", ret);
			break;
		}

		if (((block + 1U) % REPORT_BLOCKS) == 0U) {
			printk("Tone playback blocks=%u\n", block + 1U);
		}
	}

	(void)i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
	return ret;
}

static int configure_i2s_dir(const struct device *i2s_dev, enum i2s_dir dir,
			     struct k_mem_slab *mem_slab)
{
	struct i2s_config i2s_cfg = {
		.word_size = SAMPLE_BIT_WIDTH,
		.channels = CHANNEL_COUNT,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER,
		.frame_clk_freq = SAMPLE_FREQUENCY,
		.mem_slab = mem_slab,
		.block_size = BLOCK_SIZE,
		.timeout = TIMEOUT_MS,
	};
	int ret;

	ret = i2s_configure(i2s_dev, dir, &i2s_cfg);
	if (ret < 0) {
		printk("Failed to configure I2S %s: %d\n",
		       (dir == I2S_DIR_RX) ? "RX" : "TX", ret);
		return ret;
	}

	return 0;
}

static int prime_tx_path(const struct device *i2s_dev)
{
	for (uint32_t i = 0U; i < TX_PRIME_BLOCKS; ++i) {
		void *tx_block = NULL;
		int ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);

		if (ret < 0) {
			printk("Failed to allocate TX prime block: %d\n", ret);
			return ret;
		}

		memset(tx_block, 0, BLOCK_SIZE);

		ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE);
		if (ret < 0) {
			k_mem_slab_free(&tx_mem_slab, tx_block);
			printk("Failed to queue TX prime block: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int read_codec_reg(const struct device *i2c_dev, uint8_t reg, uint16_t *value)
{
	uint8_t raw[2];
	int ret;

	ret = i2c_write_read(i2c_dev, CODEC_I2C_ADDR, &reg, sizeof(reg), raw, sizeof(raw));
	if (ret < 0) {
		return ret;
	}

	*value = ((uint16_t)raw[0] << 8) | raw[1];
	return 0;
}

static void dump_codec_state(const struct device *i2c_dev)
{
	uint16_t reg = 0U;

	if (read_codec_reg(i2c_dev, WM8904_REG_POWER_MGMT_0, &reg) == 0) {
		printk("WM8904 R0C POWER_MGMT_0 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_POWER_MGMT_2, &reg) == 0) {
		printk("WM8904 R0E POWER_MGMT_2 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_POWER_MGMT_6, &reg) == 0) {
		printk("WM8904 R12 POWER_MGMT_6 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_LEFT_IN_0, &reg) == 0) {
		printk("WM8904 R2C ANALOG_LEFT_IN_0 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_RIGHT_IN_0, &reg) == 0) {
		printk("WM8904 R2D ANALOG_RIGHT_IN_0 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_LEFT_IN_1, &reg) == 0) {
		printk("WM8904 R2E ANALOG_LEFT_IN_1 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_RIGHT_IN_1, &reg) == 0) {
		printk("WM8904 R2F ANALOG_RIGHT_IN_1 = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_OUT12_ZC, &reg) == 0) {
		printk("WM8904 R3D ANALOG_OUT12_ZC = 0x%04x\n", reg);
	}

	if (read_codec_reg(i2c_dev, WM8904_REG_ANALOG_HP_0, &reg) == 0) {
		printk("WM8904 R5A ANALOG_HP_0 = 0x%04x\n", reg);
	}
}

int main(void)
{
	const struct device *const i2s_dev = DEVICE_DT_GET(I2S_CODEC_BUS);
	const struct device *const codec_i2c_dev = DEVICE_DT_GET(CODEC_I2C_BUS);
	const struct device *const codec_dev = DEVICE_DT_GET(CODEC_NODE);
	int ret;

	printk("WM8904 tone and bypass demo at %u Hz\n", SAMPLE_FREQUENCY);

	if (!device_is_ready(i2s_dev)) {
		printk("%s is not ready\n", i2s_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(codec_dev)) {
		printk("%s is not ready\n", codec_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(codec_i2c_dev)) {
		printk("%s is not ready\n", codec_i2c_dev->name);
		return -ENODEV;
	}

	printk("Tone playback mode\n");
	audio_codec_stop_output(codec_dev);
	ret = configure_codec(codec_dev, AUDIO_ROUTE_PLAYBACK);
	if (ret < 0) {
		return ret;
	}
	dump_codec_state(codec_i2c_dev);

	ret = play_tone_phase(i2s_dev);
	if (ret < 0) {
		audio_codec_stop_output(codec_dev);
		return ret;
	}

	audio_codec_stop_output(codec_dev);
	printk("Analog bypass mode on IN%u\n", capture_input);
	ret = configure_codec(codec_dev, AUDIO_ROUTE_BYPASS);
	if (ret < 0) {
		return ret;
	}

	ret = select_capture_input(codec_dev, capture_input);
	if (ret < 0) {
		audio_codec_stop_output(codec_dev);
		return ret;
	}

	dump_codec_state(codec_i2c_dev);
	printk("Bypass active on IN%u\n", capture_input);

	while (1) {
		k_msleep(1000);
	}

	return 0;
}
