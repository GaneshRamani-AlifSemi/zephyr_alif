# Copyright 2025 AlifSemiconductor.
# SPDX-License-Identifier: Apache-2.0
"""
Runner for Alif binary image burner.
"""

import argparse
import os
import sys
import re
import hashlib
import random
import shutil
from runners.core import ZephyrBinaryRunner, RunnerCaps

class AlifImageBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for Alif Image Flasher.'''

    def __init__(self, cfg, tool, sn, erase=False, reset=True):
    super().__init__(cfg)
    self.bin_ = cfg.bin_file

    self.tool = tool
    self.erase = bool(erase)
    self.reset = bool(reset)

    @classmethod
    def name(cls):
        return 'alif_flash'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, erase=True, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.set_defaults(reset=True)

    @classmethod
    def do_create(cls, cfg, args):
        return AlifImageBinaryRunner(cfg, tool=args.tool, sn=args.sn,
                                      erase=args.erase)

    def do_run(self, command, **kwargs):
        return