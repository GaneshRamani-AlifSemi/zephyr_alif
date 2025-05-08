# Copyright 2025 AlifSemiconductor.
# SPDX-License-Identifier: Apache-2.0
"""
Runner for Alif binary image burner.
"""

import argparse
import os
import sys
import re
from pathlib import Path
import json

from runners.core import ZephyrBinaryRunner, RunnerCaps

class AlifImageBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for Alif Image Flasher.'''

    home_path = str(Path.home())

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
        
        self.require(self.gen_toc)

        kwargs['cwd'] ='/home/ganesh/Alif_Tools/'

        fls_addr = self.flash_address_from_build_conf(self.build_conf)
        self.logger.info(f'binary address 0x{fls_addr:x}')

        if self.build_conf.getboolean('CONFIG_SOC_E7_DK_RTSS_HP'):
            self.logger.info("It is HighPerformance Core")
            build_core = "hp"
        else:   
            self.logger.info("It is HighEfficency Core")
            build_core = "he"

        self.prepare_json(build_core, fls_addr)

        self.check_call([self.gen_toc, '-f',
                    self.home_path +
                    '/Alif_Tools/build/config/app_cpu_stubs.json'],
                    **kwargs)

        #Write ToC.

    @classmethod
    def prepare_json(cls, build_core, fls_addr):
        
        with open(cls.home_path +'/Alif_Tools/build/config/app-cpu-stubs.json', 'r') as conf_file:
            json_data = json.load(conf_file)

        if build_core is "hp" :
            cpu_node = json_data["HP_APP"]
        else :
            cpu_node = json_data["HE_APP"]

        #update binary name
        cpu_node["binary"] = "zephyr.bin"

        #verify flash address
        if fls_addr == "0x0" and cpu_node.get('mramAdress') is not None:
            del cpu_node['mramAddress']
            if build_core is "hp":
                cpu_node["loadAddress"] = 0x50000000
            else :
                cpu_node['loadAddress'] = 0x58000000   
            cpu_node["flags"] = ["load","boot"]

        elif fls_addr != 0 and cpu_node.get('loadAddress') is not None:
            del cpu_node['loadAddress']
            cpu_node['mramAddress'] = hex(fls_addr)
            cpu_node['flags'] = ["boot"]

        with open("test.json", 'w') as file:
            json.dump(json_data, file, indent=4)
