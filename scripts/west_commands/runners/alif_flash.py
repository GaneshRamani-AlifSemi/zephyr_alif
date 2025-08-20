# Copyright 2025 AlifSemiconductor.
# SPDX-License-Identifier: Apache-2.0
"""
Runner for Alif binary image burner.
"""
import os
import json
import shutil

import fdt
import argparse
import ipaddress
import subprocess
import sys

from pathlib import Path
from runners.core import ZephyrBinaryRunner, RunnerCaps, FileType

try:
    import pylink
    from pylink.library import Library
    MISSING_REQUIREMENTS = False
except ImportError:
    MISSING_REQUIREMENTS = True

DEFAULT_JLINK_GDB_PORT = 2331
DEFAULT_JLINK_GDB_SERVER = 'JLinkGDBServer'


class AlifImageBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for Alif Image Flasher.'''

    #Location.
    zephyr_repo = str(Path.cwd())

    exe_dir = os.getenv("ALIF_SE_TOOLS_DIR")

    mram_base_addr = int('0x80000000', 0)

    cfg_ip_file = '/build/config/app-cpu-stubs.json'
    cfg_op_file = '/build/config/tmp-cpu-stubs.json'


    def __init__(self, cfg, device,
                 erase=False, reset=True,
                 iface='swd', speed='auto',
                 gdbserver=DEFAULT_JLINK_GDB_SERVER,
                 gdb_port=DEFAULT_JLINK_GDB_PORT,
                 toc_create='app-gen-toc', toc_write='app-write-mram'):
        super().__init__(cfg)
        self.bin_ = cfg.bin_file

        self.gen_toc = toc_create
        self.write_toc = toc_write
        self.gdbserver = gdbserver
        self.gdb_port = gdb_port
        self.device = device
        self.iface = iface
        self.speed = speed
        self.erase = bool(erase)
        self.reset = bool(reset)
        self.gdb_cmd = [cfg.gdb] if cfg.gdb else None
        self.file = cfg.file
        self.file_type = cfg.file_type
        self.hex_name = cfg.hex_file
        self.bin_name = cfg.bin_file
        self.elf_name = cfg.elf_file


    @classmethod
    def name(cls):
        return 'alif_flash'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash', 'debug', 'attach'}, erase=False, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        # Required:
        parser.add_argument('--device', required=True, help='device name')

        # Optional:
        parser.add_argument('--iface', default='swd',
                            help='interface to use, default is swd')
        parser.add_argument('--speed', default='auto',
                            help='interface speed, default is autodetect')
        parser.add_argument('--tui', default=False, action='store_true',
                            help='if given, GDB uses -tui')
        parser.add_argument('--gdbserver', default='JLinkGDBServer',
                            help='GDB server, default is JLinkGDBServer')
        parser.add_argument('--gdb-port', default=DEFAULT_JLINK_GDB_PORT,
                            help='pyocd gdb port, defaults to {}'.format(
                                DEFAULT_JLINK_GDB_PORT))

        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        return AlifImageBinaryRunner(cfg, args.device,
                                    erase=args.erase,
                                    reset=args.reset,
                                    iface=args.iface, speed=args.speed,
                                    gdb_port=args.gdb_port)

    @property
    def jlink_version(self):
        # Get the J-Link version as a (major, minor, rev) tuple of integers.
        #
        # J-Link's command line tools provide neither a standalone
        # "--version" nor help output that contains the version. Hack
        # around this deficiency by using the third-party pylink library
        # to load the shared library distributed with the tools, which
        # provides an API call for getting the version.
        if not hasattr(self, '_jlink_version'):
            # pylink 0.14.0/0.14.1 exposes JLink SDK DLL (libjlinkarm) in
            # JLINK_SDK_STARTS_WITH, while other versions use JLINK_SDK_NAME
            if pylink.__version__ in ('0.14.0', '0.14.1'):
                sdk = Library.JLINK_SDK_STARTS_WITH
            else:
                sdk = Library.JLINK_SDK_NAME

            plat = sys.platform
            if plat.startswith('win32'):
                libname = Library.get_appropriate_windows_sdk_name() + '.dll'
            elif plat.startswith('linux'):
                libname = sdk + '.so'
            elif plat.startswith('darwin'):
                libname = sdk + '.dylib'
            else:
                self.logger.warning(f'unknown platform {plat}; assuming UNIX')
                libname = sdk + '.so'

            lib = Library(dllpath=os.fspath(Path(self.commander).parent /
                                            libname))
            version = int(lib.dll().JLINKARM_GetDLLVersion())
            self.logger.debug('JLINKARM_GetDLLVersion()=%s', version)
            # The return value is an int with 2 decimal digits per
            # version subfield.
            self._jlink_version = (version // 10000,
                                   (version // 100) % 100,
                                   version % 100)

        return self._jlink_version

    @property
    def jlink_version_str(self):
        # Converts the numeric revision tuple to something human-readable.
        if not hasattr(self, '_jlink_version_str'):
            major, minor, rev = self.jlink_version
            rev_str = chr(ord('a') + rev - 1) if rev else ''
            self._jlink_version_str = f'{major}.{minor:02}{rev_str}'
        return self._jlink_version_str

    @property
    def supports_nogui(self):
        # -nogui was introduced in J-Link Commander v6.80
        return self.jlink_version >= (6, 80, 0)

    @property
    def supports_thread_info(self):
        # RTOSPlugin_Zephyr was introduced in 7.11b
        return self.jlink_version >= (7, 11, 2)

    @property
    def supports_loader(self):
        return self.jlink_version >= (7, 70, 4)

    def flash(self, **kwargs):

        #check env
        if self.exe_dir is None:
            raise RuntimeError("ALIF_SE_TOOLS_DIR Environment unset")

        self.logger.info("GenToc bin name %s Exedir %s", self.gen_toc, self.exe_dir)
        #check tools availability
        self.require(self.gen_toc, self.exe_dir)

        fls_addr = self.flash_address_from_build_conf(self.build_conf)
        fls_size = self.build_conf.get('CONFIG_FLASH_SIZE')

        self.logger.info("binary address %s and size %s KB", hex(fls_addr), fls_size)

        if self.build_conf.getboolean('CONFIG_RTSS_HP'):
            self.logger.info("..build for HighPerformance Core")
            build_core = "hp"
        else:
            self.logger.info("..build for HighEfficency Core")
            build_core = "he"

        shutil.copy(
            os.path.join(self.cfg.build_dir, 'zephyr', 'zephyr.bin'),
            os.path.join(self.exe_dir, 'build/images/')
        )


        # change dir just for the JSON and TOC
        old_cwd = os.getcwd()
        try:
            os.chdir(self.exe_dir)
            self.logger.info(f"Changed working directory to {self.exe_dir}")

            #Prepare json to create AToC.
            self.prepare_json(self.logger, build_core, fls_addr, fls_size)

            # Generate ToC.
            self.check_call(['./' + self.gen_toc, '-f',
                             self.exe_dir + self.cfg_op_file],
                            **kwargs)

            # Write ToC.
            self.check_call(['./' + self.write_toc, '-p'], **kwargs)

        finally:
            os.chdir(old_cwd)
            self.logger.info(f"Returned to working directory {old_cwd}")

    def debug(self, **kwargs):
            if MISSING_REQUIREMENTS:
                raise RuntimeError('one or more Python dependencies were missing; '
                               "see the getting started guide for details on "
                               "how to fix")

            server_cmd = ([self.gdbserver] +
                      ['-select', 'usb',
                       '-port', str(self.gdb_port),
                       '-if', self.iface,
                       '-speed', self.speed,
                       '-device', self.device,
                       '-silent',
                       '-singlerun', '-nogui'])

            if self.gdb_cmd is None:
                raise ValueError('Cannot debug; gdb is missing')
            if self.file is not None:
                if self.file_type != FileType.ELF:
                    raise ValueError('Cannot debug; elf file required')
                elf_name = self.file
            elif self.elf_name is None:
                raise ValueError('Cannot debug; elf is missing')
            else:
                elf_name = self.elf_name
            client_cmd = (self.gdb_cmd +
                          [elf_name] +
                          ['-ex', 'target remote :{}'.format(self.gdb_port)] +
                          ['-ex', 'monitor halt'] +
                          ['-ex', 'monitor reset'])

            self.require(self.gdbserver)
            self.logger.info('Alif : J-Link GDB server port : server_cmd : client_cmd '
                        f'{self.gdb_port}{server_cmd}{client_cmd}')
            self.run_server_and_client(server_cmd, client_cmd)



    def do_run(self, command, **kwargs):

        #Even it is Debug make sure binary flashed in MRAM.
        if command == 'flash':
            self.flash(**kwargs)

        if command == 'debug':
            self.debug(**kwargs)

    @classmethod
    def get_itcm_address(cls, logger):
        """Function retrieves itcm address."""
        try:
            with open(os.path.join(cls.zephyr_repo,
                                   'build', 'zephyr', 'zephyr.dts'), "r", encoding="utf-8") as f:
                dtext = f.read()
        except Exception as err:
            logger.error(f"dts parsing error {err}")
            return 0

        try:
            dt2 = fdt.parse_dts(dtext)
            addr = dt2.get_node('soc').get_subnode('itcm@0').get_property('itcm_global_base')
        except Exception as err:
            logger.error(f"Parsing itcm address {err}")
            return 0
        return addr.value

    @classmethod
    def prepare_json(cls, logger, build_core, fls_addr, fls_size):
        """prepares json"""
        try:
            with open(cls.exe_dir + cls.cfg_ip_file, 'r', encoding="utf-8") as conf_file:
                json_data = json.load(conf_file)
        except IOError as err:
            logger.error(f"Can't open file to read {cls.exe_dir + cls.cfg_ip_file} : {err}")
            return

        if build_core == "hp":
            cpu_node = json_data["HP_APP"]
        else:
            cpu_node = json_data["HE_APP"]

        #update binary name
        cpu_node["binary"] = "zephyr.bin"

        #verify flash address
        if fls_addr == 0:
            itcm_addr = cls.get_itcm_address(logger)
            if itcm_addr == 0:
                logger.error(f"err addr 0x{itcm_addr:x}")
                return
            logger.info(f"itcm global address 0x{itcm_addr:x}")
            cpu_node["loadAddress"] = hex(itcm_addr)
            cpu_node["flags"] = ["load","boot"]

        elif fls_addr >= cls.mram_base_addr and fls_addr <= cls.mram_base_addr + (fls_size * 1024):
            del cpu_node['loadAddress']
            cpu_node['mramAddress'] = hex(fls_addr)
            cpu_node['flags'] = ["boot"]

        else:
            raise NotImplementedError(f'Unsupported address base 0x{fls_addr:x} to write')

        try:
            with open(cls.exe_dir + cls.cfg_op_file, 'w', encoding="utf-8") as file:
                json.dump(json_data, file, indent=4)
        except IOError as err:
            logger.error(f"Can't open file to write {cls.exe_dir + cls.cfg_op_file}: {err}")
