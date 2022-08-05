"""
Copyright (c) 2022, Jose Luis Cercos-Pita <jlc@core-marine.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# This file is used to build self-contained Python wrappers, which does not
# depends on the prebuilt libmoordyn.so.2. This is ideal to be used within
# cibuildwheel, which is producing the artifacts that can be uploaded later to
# Pipy
# In general you do not want to use this script by your own. It is meant for the
# core developers to setup the pipy packages

import os
from setuptools import setup, find_packages, Extension
import sysconfig


# Collect the source code files
MOORDYN_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'source')
MOORDYN_SRCS = []
for f in os.listdir(MOORDYN_PATH):
    if not os.path.isfile(os.path.join(MOORDYN_PATH, f)):
        continue
    if not f.lower().endswith(".c") and not f.lower().endswith(".cpp"):
        continue
    MOORDYN_SRCS.append(os.path.join('source', f))
MOORDYN_SRCS.append(os.path.join('wrappers', 'python', 'cmoordyn.cpp'))

# Read the version from the CMakeLists.txt
version = ""
with open('CMakeLists.txt', 'r') as f:
    txt = f.read()
    for name in ('MAJOR', 'MINOR', 'PATCH'):
        prefix = 'set(MOORDYN_{}_VERSION '.format(name)
        subtxt = txt[txt.find(prefix) + len(prefix):]
        subtxt = subtxt[:subtxt.find(')\n')]
        version = version + subtxt + '.'
    version = version[:-1]

cmoordyn = Extension('cmoordyn',
                     sources=MOORDYN_SRCS,
                     language='c++',
                     include_dirs=[MOORDYN_PATH, ],
                     define_macros=[('MoorDyn_EXPORTS', 'None')],
                     )

setup(
    name='moordyn',
    version=version,
    description='MoorDyn for Python',
    author='Jose Luis Cercos-Pita',
    author_email='jlc@core-marine.com',
    ext_modules = [cmoordyn],
    packages=find_packages(include=['moordyn', 'moordyn.*']),
    install_requires=[],
    setup_requires=['cython', 'pytest-runner'],
    tests_require=['pytest'],
)