.. zephyr:code-sample:: i2s-codec-passthrough
   :name: I2S codec passthrough
   :relevant-api: i2s_interface audio_codec

   Play a short I2S tone, then switch the WM8904 to analog bypass.

Overview
********

This sample configures the WM8904 to play a short tone over I2S once, then
switches the codec into analog bypass mode on input ``IN2`` and stays there.

The application:

- configures the WM8904 for playback during the tone phase,
- streams a short tone over I2S,
- switches the WM8904 to bypass mode after the tone phase,
- selects capture input ``IN2``, and
- idles in bypass mode once the switch is complete.

Requirements
************

This sample expects:

- a board with a WM8904 audio codec,
- the ``i2s-codec`` snippet enabled at build time, and
- a line-level analog source for the bypass phase.

Building and Running
********************

The code can be found in
:zephyr_file:`samples/drivers/i2s/codec_passthrough`.

To build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/i2s/codec_passthrough
   :board: alif_b1_dk/ab1c1f4m51820hh0/rtss_he
   :goals: build flash
   :gen-args: -DSNIPPET=i2s-codec
   :compact:

If you are using a different board, replace the board name with the matching
WM8904-capable board and keep the ``i2s-codec`` snippet enabled.
