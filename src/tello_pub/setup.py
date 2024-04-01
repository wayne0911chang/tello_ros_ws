#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['tello_driver'],
     # for laptop dir
     package_dir={'': '/home/chungyu/wayne-temp/20220901_tello_ws/src/tello_driver'}
)

setup(**setup_args)
