#!/usr/bin/env python
# -*- encoding: utf-8 -*-
# flake8: noqa
from __future__ import absolute_import
from __future__ import print_function

import io
from os import path

from setuptools import setup
from setuptools.command.develop import develop
from setuptools.command.install import install


requirements = [
    # Until COMPAS reaches 1.0, we pin major.minor and allow patch version updates
    'compas>=0.16.9',
    'compas_fab>=0.12.0',
    # 'compas>=0.17.2,<0.18',
    # 'compas_fab>=0.13.1',
    'pybullet_planning>=0.5.1',
    'ikfast_pybind>=0.1.0'
]
# 'compas_fab @ git+https://git@github.com/compas-dev/compas_fab.git@5163c5619d12e102f5e070731c7f21c139eb4549#egg=compas_fab-0.11.1',

here = path.abspath(path.dirname(__file__))

def read(*names, **kwargs):
    return io.open(
        path.join(here, *names),
        encoding=kwargs.get("encoding", "utf8")
    ).read()

about = {}
exec(read('src', 'compas_fab_pychoreo', '__version__.py'), about)

long_description = read("README.md")
# requirements = read("requirements.txt").split("\n")
optional_requirements = {}

setup(
    name=about['__title__'],
    version=about['__version__'],
    license=about['__license__'],
    description=about['__description__'],
    author=about['__author__'],
    author_email=about['__author_email__'],
    url=about['__url__'],
    # name="compas_fab_pychoreo",
    # version="0.2.0",
    # description="an experimental package containing research prototpyes to integrate more planning functionalities to compas_fab",
    # url="https://github.com/yijiangh/compas_fab_pychoreo",
    # author="Yijiang Huang",
    # author_email="yijiangh@mit.edu",
    # license="MIT license",
    long_description=long_description,
    long_description_content_type="text/markdown",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Unix",
        "Operating System :: POSIX",
        "Operating System :: Microsoft :: Windows",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
    keywords=[],
    project_urls={},
    packages=["compas_fab_pychoreo"],
    package_dir={"": "src"},
    package_data={},
    data_files=[],
    include_package_data=True,
    zip_safe=False,
    install_requires=requirements,
    python_requires=">=3",
    extras_require=optional_requirements,
    entry_points={
        "console_scripts": [],
    },
    ext_modules=[],
)
