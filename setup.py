#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['topic_store'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'rospkg', 'bson', 'pymongo']
)

setup(**setup_args)
