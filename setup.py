#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ceiling_cam_fiducials_odom_ekf'],
    package_dir={'': 'scripts'},
    )

setup(**d)
