#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bt_audio', 'bt_apiai_service', 'bt_utils'],
    package_dir={'': 'src'}
)

setup(**d)
