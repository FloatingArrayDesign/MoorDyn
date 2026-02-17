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
import shutil
from setuptools import setup, find_packages, Extension
import platform


# Collect the source code files
def get_sources(folder, sources=[]):
    for f in os.listdir(folder):
        if os.path.isdir(os.path.join(folder, f)):
            sources += get_sources(os.path.join(folder, f), sources=[])
        if not os.path.isfile(os.path.join(folder, f)):
            continue
        if not f.lower().endswith(".c") and not f.lower().endswith(".cpp"):
            continue
        sources.append(os.path.join(folder, f))
    return sources


# Check for needed files
def arefiles(files):
    return all([os.path.isfile(f) for f in files])

MODULE_PATH = os.path.join('wrappers', 'python', 'moordyn')
MOORDYN_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'source')
MOORDYN_SRCS = [os.path.join('wrappers', 'python', 'cmoordyn.cpp')]

if os.path.isdir(MODULE_PATH):
    # We better copy the moordyn module on the root
    shutil.rmtree('moordyn', ignore_errors=True)
    shutil.copytree(MODULE_PATH, 'moordyn')

# Eigen needs at least C++ 14, and Moordyn itself uses C++ 17
extra_compile_args = ["-std=c++17"]
if platform.system() == "Windows":
    extra_compile_args.append("/std:c++17")
elif platform.system() == "Darwin":
    # To avoid errors with std::filesystem::path
    extra_compile_args.append("-mmacosx-version-min=10.15")
definitions = [('MoorDyn_EXPORTS', '1'),
               ('MOORDYN_MAJOR_VERSION', '${MOORDYN_MAJOR_VERSION}'),
               ('MOORDYN_MINOR_VERSION', '${MOORDYN_MINOR_VERSION}'),
               ('MOORDYN_PATCH_VERSION', '${MOORDYN_PATCH_VERSION}')]
include_dirs = [MOORDYN_PATH, 'install/include/moordyn']
library_dirs = ['install/lib/', 'install/lib64/']
libraries = ['moordyn']

cmoordyn = Extension('cmoordyn',
                     sources=MOORDYN_SRCS,
                     language='c++',
                     define_macros=definitions,
                     include_dirs=include_dirs,
                     library_dirs=library_dirs,
                     libraries=libraries,
                     extra_compile_args=extra_compile_args,
                     )

setup(
    packages=find_packages(include=['moordyn', 'moordyn.*']),
    ext_modules=[cmoordyn],
    package_data={"source": ["*.hpp", "*.h"],
                  "source.Util": ["*.hpp", "*.h"],
                  "source.Waves": ["*.hpp", "*.h"],
                  ${EIGEN_PACKAGE_DATA}}
)
