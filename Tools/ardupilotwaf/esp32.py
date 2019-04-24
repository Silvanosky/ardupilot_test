#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for ESP32 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle
import subprocess

def parse_inc_dir(lines):
    for line in lines.splitlines():
        if line.startswith('INCLUDES: '):
            return line.replace('INCLUDES: ', '').split()

def configure(cfg):
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    cfg.find_program('make', var='MAKE')
    env = cfg.env
    env.AP_HAL_PLANE = srcpath('libraries/AP_HAL_ESP32/targets/plane')
    env.AP_HAL_COPTER = srcpath('libraries/AP_HAL_ESP32/targets/copter')
    env.AP_HAL_ROVER = srcpath('libraries/AP_HAL_ESP32/targets/rover')
    env.AP_HAL_SUB = srcpath('libraries/AP_HAL_ESP32/targets/sub')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']
    cmd = "cd {0}&&echo '{2}' > ../board.txt&&{1} defconfig BATCH_BUILD=1&&{1} showinc BATCH_BUILD=1".format(env.AP_HAL_PLANE, env.MAKE[0], env.BOARD)
    result = subprocess.check_output(cmd, shell=True)
    env.INCLUDES += parse_inc_dir(result)

class build_esp32_image_plane(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_PLANE}&&'${MAKE}' BATCH_BUILD=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_copter(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_COPTER}&&'${MAKE}' BATCH_BUILD=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_rover(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_ROVER}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_esp32_image_sub(Task.Task):
    '''build an esp32 image'''
    color='CYAN'
    run_str="cd ${AP_HAL_SUB}&&'${MAKE}' V=1"
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    if str(self.link_task.outputs[0]).endswith('libarduplane.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduPlane_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarduplane.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-plane/arduplane.elf')
        generate_bin_task = self.create_task('build_esp32_image_plane', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)

    if str(self.link_task.outputs[0]).endswith('libarducopter.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduCopter_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libarducopter.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-copter/arducopter.elf')
        generate_bin_task = self.create_task('build_esp32_image_copter', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)

    if str(self.link_task.outputs[0]).endswith('libardurover.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libAPMrover2_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libardurover.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-rover/ardurover.elf')
        generate_bin_task = self.create_task('build_esp32_image_rover', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)

    if str(self.link_task.outputs[0]).endswith('libardusub.a'):
        #build final image
        src_in = [self.bld.bldnode.find_or_declare('lib/libArduSub_libs.a'),
                  self.bld.bldnode.find_or_declare('lib/bin/libardusub.a')]
        img_out = self.bld.bldnode.find_or_declare('idf-sub/ardusub.elf')
        generate_bin_task = self.create_task('build_esp32_image_sub', src=src_in, tgt=img_out)
        generate_bin_task.set_run_after(self.link_task)
