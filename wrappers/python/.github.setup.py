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
import sysconfig
import platform


DESC = """Python version of MoorDyn, a lumped-mass mooring dynamics model
intended for coupling with floating structure codes.

This is not a wrapper of MoorDyn, but an stand-alone version. Thus you can
just install and use this package, no need of the MoorDyn library in your
system.

Please, visit https://moordyn-v2.readthedocs.io to learn more about this package,
and MoorDyn in general.

If you detect any problem, please feel free to report the issue on the GitHub
page:
https://github.com/mattEhall/MoorDyn

WARNING: This is a pre-release version of MoorDyn v2. The API may be changed
in the future, breaking your program
"""

# We better copy here the moordyn module
shutil.rmtree('moordyn', ignore_errors=True)
shutil.copytree(os.path.join('wrappers', 'python', 'moordyn'), 'moordyn')


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


MOORDYN_PATH = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                            'source')
MOORDYN_SRCS = get_sources('source')
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

# Get everything required to compile with VTK support
vtk_version = '9.2'
try:
    vtk_version = os.environ['VTK_VERSION_MAJOR'] + "." + \
        os.environ['VTK_VERSION_MINOR']
except KeyError:
    print("$VTK_VERSION_MAJOR.$VTK_VERSION_MINOR env variables missing")
vtk_libraries = ["vtkCommonCore", "vtkIOXML", "vtkIOGeometry",
                 "vtkIOXMLParser", "vtkIOLegacy", "vtkIOCore",
                 "vtkCommonExecutionModel", "vtkCommonDataModel",
                 "vtkCommonTransforms", "vtkCommonMath", "vtkCommonMisc",
                 "vtkCommonSystem", "vtkFiltersGeneral", "vtkFiltersCore",
                 "vtkdoubleconversion", "vtklz4", "vtklzma", "vtkzlib",
                 "vtkkissfft", "vtkpugixml", "vtkexpat", "vtkloguru", "vtksys"]

# Eigen needs at least C++ 14, and Moordyn itself uses C++ 17
extra_compile_args = ["-std=c++17"]
if platform.system() == "Windows":
    extra_compile_args.append("/std:c++17")
elif platform.system() == "Darwin":
    # To avoid errors with std::filesystem::path
    extra_compile_args.append("-mmacosx-version-min=10.15")
definitions = [('MoorDyn_EXPORTS', '1'), ('USE_VTK', '1')]
include_dirs = [MOORDYN_PATH, "vtk/include/vtk-" + vtk_version]
library_dirs = ["vtk/lib"]
if platform.system() == "Windows":
    extra_link_args = ["vtk/lib/" + lib + "-" + vtk_version + ".lib"
                       for lib in vtk_libraries]
    extra_link_args = extra_link_args + [
        "ws2_32.lib", "dbghelp.lib", "psapi.lib", "kernel32.lib", "user32.lib",
        "gdi32.lib", "winspool.lib", "shell32.lib", "ole32.lib",
        "oleaut32.lib", "uuid.lib", "comdlg32.lib", "advapi32.lib"]
else:
    extra_link_args = ["vtk/lib/lib" + lib + "-" + vtk_version + ".a"
                       for lib in vtk_libraries]
             
cmoordyn = Extension('cmoordyn',
                     sources=MOORDYN_SRCS,
                     language='c++',
                     define_macros=definitions,
                     include_dirs=include_dirs,
                     extra_compile_args=extra_compile_args,
                     extra_link_args=extra_link_args,
                     )

setup(
    name='moordyn',
    version=version,
    author='Jose Luis Cercos-Pita',
    author_email='jlc@core-marine.com',
    url = 'https://github.com/mattEhall/MoorDyn',
    description='MoorDyn for Python',
    long_description = DESC,
    ext_modules = [cmoordyn],
    packages=find_packages(include=['moordyn', 'moordyn.*']),
    install_requires=[],
    setup_requires=['cython', 'pytest-runner'],
    tests_require=['pytest'],
)
