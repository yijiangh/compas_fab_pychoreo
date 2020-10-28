"""
********************************************************************************
compas_fab_pychoreo
********************************************************************************

.. currentmodule:: compas_fab_pychoreo


.. toctree::
    :maxdepth: 1


"""

from __future__ import print_function

import os
import sys


__author__ = ["Yijiang Huang"]
__copyright__ = "Yijiang Huang"
__license__ = "MIT License"
__email__ = "yijiangh@mit.edu"
__version__ = "__version__ = '0.3.0'"


HERE = os.path.dirname(__file__)

HOME = os.path.abspath(os.path.join(HERE, "../../"))
DATA = os.path.abspath(os.path.join(HOME, "data"))
DOCS = os.path.abspath(os.path.join(HOME, "docs"))
TEMP = os.path.abspath(os.path.join(HOME, "temp"))

# Check if package is installed from git
# If that's the case, try to append the current head's hash to __version__
try:
    import compas
    git_head_file = compas._os.absjoin(HOME, '.git', 'HEAD')

    if os.path.exists(git_head_file):
        # git head file contains one line that looks like this:
        # ref: refs/heads/master
        with open(git_head_file, 'r') as git_head:
            _, ref_path = git_head.read().strip().split(' ')
            ref_path = ref_path.split('/')

            git_head_refs_file = compas._os.absjoin(HOME, '.git', *ref_path)

        if os.path.exists(git_head_refs_file):
            with open(git_head_refs_file, 'r') as git_head_ref:
                git_commit = git_head_ref.read().strip()
                __version__ += '-' + git_commit[:8]
except Exception:
    pass

__all__ = ["HOME", "DATA", "DOCS", "TEMP"]
