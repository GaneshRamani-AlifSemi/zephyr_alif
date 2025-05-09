# Copyright 2025 AlifSemiconductor.
# SPDX-License-Identifier: Apache-2.0
"""
Runner for Alif binary image burner.
"""

import sys
import json

from pathlib import Path
from runners.core import ZephyrBinaryRunner, RunnerCaps

class AlifImageBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for Alif Image Flasher.'''

    #Address and Location properties.
    home_path = str(Path.home())
    zephyr_repo = str(Path.cwd())
    hp_itcm_load_address = '0x50000000'
    he_itcm_load_address = '0x58000000'
    he_mram_boot_address = '0x80000000'
    hp_mram_boot_address = '0x80200000'
    cfg_ip_file = '/Alif_Tools/build/config/app-cpu-stubs.json'
    cfg_op_file = '/Alif_Tools/build/config/tmp-app-cpu.json'
    exe_dir = home_path + '/Alif_Tools'

    def __init__(self, cfg, toc_create='app-gen-toc', toc_write='app-write-mram',
                 erase=False, reset=True):
        super().__init__(cfg)
        self.bin_ = cfg.bin_file

        self.gen_toc = toc_create
        self.write_toc = toc_write
        self.erase = bool(erase)
        self.reset = bool(reset)

    @classmethod
    def name(cls):
        return 'alif_flash'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, erase=False, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        return AlifImageBinaryRunner(cfg)


    def do_run(self, command, **kwargs):
        if sys.platform == 'win32':
            self.logger.info(f'Windows not yet Supported')
            return
        
        #check tools availability
        self.require(self.gen_toc, self.exe_dir)

        #set Execution directory
        kwargs['cwd'] = self.exe_dir

        fls_addr = self.flash_address_from_build_conf(self.build_conf)
        fls_addr = f'0x{fls_addr:x}'

        self.logger.info(f'binary address {fls_addr}')

        if self.build_conf.getboolean('CONFIG_RTSS_HP'):
            self.logger.info("..build for HighPerformance Core")
            build_core = "hp"
        else:   
            self.logger.info("..build for HighEfficency Core")
            build_core = "he"

        #copy zephyr binary
        self.check_call( ['cp',
                          self.zephyr_repo + '/build/zephyr/zephyr.bin',
                          self.exe_dir + '/build/images/'])

        #Prepare json to create AToC.
        self.prepare_json(self.logger, build_core, fls_addr)

        #Generte ToC
        self.check_call(['./' + self.gen_toc, '-f',
                        self.home_path +
                        self.cfg_op_file],
                        **kwargs)

        #Write ToC with sudo access.
        self.check_call(['sudo', './' + self.write_toc, '-p'], **kwargs)

    @classmethod
    def prepare_json(cls, logger, build_core, fls_addr):
        try:
            with open(cls.home_path + cls.cfg_ip_file, 'r') as conf_file:
                json_data = json.load(conf_file)
        except IOError as err:
            logger.error(f"Can't open file to read {cls.home_path + cls.cfg_ip_file}")

        if build_core == "hp" :
            cpu_node = json_data["HP_APP"]
        else :
            cpu_node = json_data["HE_APP"]

        #update binary name
        cpu_node["binary"] = "zephyr.bin"

        #verify flash address
        if fls_addr == '0x0':
            if build_core == "hp":
                cpu_node["loadAddress"] = cls.hp_itcm_load_address
            else :
                cpu_node['loadAddress'] = cls.he_itcm_load_address
            cpu_node["flags"] = ["load","boot"]

        elif fls_addr == cls.he_mram_boot_address or fls_addr == cls.hp_mram_boot_address:
            del cpu_node['loadAddress']
            cpu_node['mramAddress'] = fls_addr
            cpu_node['flags'] = ["boot"]

        else:
            raise NotImplementedError('Invalid Address')

        try:
            with open(cls.home_path + cls.cfg_op_file, 'w') as file:
                json.dump(json_data, file, indent=4)
        except IOError as err:
            logger.error(f"Can't open file to write {cls.home_path + cls.cfg_op_file}")