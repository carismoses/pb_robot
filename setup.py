#!/usr/bin/env python
from setuptools import setup, find_packages
from setuptools.command.install import install
import subprocess
from catkin_pkg.python_setup import generate_distutils_setup
import os

d = generate_distutils_setup(
            packages=find_packages('src'),
            package_dir={'': 'src'},
)

# install dependencies
with open('requirements.txt') as f:
    install_requires = f.read().splitlines()

# setup.py to build panda ikFast module
class InstallLocalPackage(install):
    def run(self):
        install.run(self)
        subprocess.call(
            "python src/pb_robot/ikfast/franka_panda/setup.py install", shell=True
        )
        
setup(install_requires=install_requires, cmdclass={ 'install': InstallLocalPackage }, **d)