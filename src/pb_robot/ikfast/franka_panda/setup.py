#!/usr/bin/env python2

from __future__ import print_function

import sys
import os
sys.path.append('src/pb_robot/ikfast')

from compile import compile_ikfast

# Build C++ extension by running: 'python setup.py'
# see: https://docs.python.org/3/extending/building.html

def main():
    # lib name template: 'ikfast_<robot name>'
    sys.argv[:] = sys.argv[:1] + ['build']
    robot_name = 'panda_arm'
    compile_ikfast(module_name='ikfast_panda_arm',
                   cpp_filename='src/pb_robot/ikfast/franka_panda/ikfast_panda_arm.cpp')

if __name__ == '__main__':
    main()
