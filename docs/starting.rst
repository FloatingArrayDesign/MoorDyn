.. _starting:

Getting Started
===============

The latest version of MoorDyn v2 is now available on
`GitHub <https://github.com/mattEhall/moordyn/>`_.
Here it is describe both, how to install MoorDyn
:ref:`using the distributed binaries <_starting_binaries>`,
and how to :ref:`compile by yourself <_starting_compile>`.

Binaries and installers
-----------------------

.. _starting_binaries:

Two different cases must be considered when installing binaries, the
:ref:`C/C++ libraries <_starting_binaries_lib>` (with the Fortran wrapper), and
the :ref:`Python package <_starting_binaries_python>`.

C/C++ Library
^^^^^^^^^^^^^

.. _starting_binaries_lib:

To install the C/C++ library (and the Fortran wrappers), please visit the
`Releases page <https://github.com/mattEhall/MoorDyn/releases>`_, and select the
version you want to install.
Along this line you would probably want to consider either the latest version
identified with a number, or the one named "nightly".
The former is the latest stable version, while the latter is the latest version
uploaded to the repository, which tends to be a bit less stable.

Once you already chosen a release, click on the assets and select the most
appropriate one for your platform.
More specifically, if you are in Windows you probably want to download and
execute Moordyn-X.Y.Z-win64.exe (with X.Y.Z replaced by the specific version),
in Linux you can download and execute Moordyn-X.Y.Z-Linux.sh and
in MacOS you can download and execute Moordyn-X.Y.Z-Darwin.sh.

NOTE: When you donwload the self-extracting files for Linux and MacOS they
cannot be launched until you give them execution permissions.

Now you can checkout
:ref:`how to integrate MoorDyn in your project <_starting_using>` below.

Python package
^^^^^^^^^^^^^^

.. _starting_binaries_python:

Installing the Python package is extremelly easy, just type

.. code-block:: bash

  python -m pip install moordyn

in your system terminal. Pip will take care of everything by you.

Compile and install MoorDyn
---------------------------

.. _starting_compile:

The process is sligthly different in Windows than in the other platforms, so it
will be documented separatelly

Windows
^^^^^^^

In this tutorial we will install Eigen3 and MoorDyn in the default folders
(C:\Program Files (x86)\Eigen3 and C:\Program Files (x86)\Moordyn).
We will acquire the latest versions available with Git and build them
using CMake. This documentation assumes that you are building
in an MSYS2 build environment.

Please, install all the tools we need:

* `Git <https://git-scm.com/>`_
* `CMake <https://cmake.org/>`_
* `MSYS2 <https://www.msys2.org/>`_

During the installation of Git, please check that you install all the components
shown below, and add it to the PATH:

.. figure:: win_git_install.png
   :alt: Installing Git in Windows

   Recommended options while installing Git in Windows

The same holds for CMake:

.. figure:: win_cmake_install.png
   :alt: Installing CMake in Windows

   Recommended options while installing CMake in Windows

The installation of MSYS2 is pretty well documented in
`the project web page <https://www.msys2.org/>`_. However, we need some
additional tools, so after running "MSYS MinGW 64-bit", please type
the following command

.. code-block:: bash

  pacman -S mingw-w64-x86_64-python-setuptools mingw-w64-x86_64-python-pip mingw64/mingw-w64-x86_64-make mingw-w64-x86_64-gcc mingw-w64-x86_64-gdb mingw-w64-x86_64-cmake

Now we need to make the MinGW stack available across the whole system by adding
it to the PATH environment variable.
To this end, execute "System" from the Windows Init menu, and look for
"environment".
Then click on "Edit the system environment variables" and in the popping up
window on "Enviroment Variables..."
Double click on Path (in the System variables box), and add a new entry:
"C:\msys64\mingw64\bin"

.. figure:: win_msys2_env.png
   :alt: Adding MinGW to the PATH

   Adding MinGW to the PATH

Now we are ready to work! First we must create a folder where we will
download and compile the MoorDyn code, let's say C:\MoorDyn.
Create such a folder, and right click inside, selecting "Git GUI Here". In
the Git window select "Clone Existing Repository".

.. figure:: win_git_gui.png
   :alt: Git GUI in Windows

   The Git GUI to clone repositories

We are starting with Eigen3, so in the first box of the window that pops up set
"https://gitlab.com/libeigen/eigen.git", and in the second "C:\MoorDyn\eigen":

.. figure:: win_git_eigen.png
   :alt: Options to clone Eigen3

   Cloning Eigen3 repository

Press "Clone" and let Git download the repository.
Now you can repeat, setting "https://github.com/mattEhall/MoorDyn.git", and
"C:\MoorDyn\MoorDyn" to download MoorDyn:

.. figure:: win_git_moordyn.png
   :alt: Options to clone MoorDyn

   Cloning MoorDyn repository

Now, create two additional folders in C:\MoorDyn named eigen.build and
MoorDyn.build. As suggested by the names, these folders are where we will
actually build the source code we just cloned from GitHub. To do this, we'll
be using CMake as our build tool.

Start CMake from the Windows Init menu. To prepare Eigen3 set
"C:\MoorDyn\eigen" in the source box and "C:\MoorDyn\eigen.build" in the
binaries box, and press "Configure".
The first time you configure a new project, CMake will ask you for the toolchain
to use. Select "MinGW Makefiles":

.. figure:: win_cmake_selectcompiler.png
   :alt: Selecting the MinGW generator

   Selecting the MinGW toolchain as generator

Click on "Finish" and let CMake work. After a short while you will see a lot of
new red boxes.
Don't worry, these are not errors - they are red because they are new, and you
must specify some additional parameters for CMake.
Remember to set CMAKE_BUILD_TYPE as "Release" (unless you are working on the
source code, in which case you may wish to set the build type to "Debug" so
as to run the built program through a debugger).
It is also recommended to disable BUILD_TESTING, EIGEN_BUILD_DOC and
EIGEN_BUILD_TESTING:

.. figure:: win_cmake_eigen.png
   :alt: Configuration options for Eigen3

   Configuration options for Eigen3

Press "Configure" once again, and then "Generate". Now you can close CMake.

Now, since we are installing Eigen in C:\Program Files (x86)\Eigen3, we need
to execute a Command Prompt with administrative rights.
Search for "cmd" in the Windows Init menu and right click on
"Command Prompt", selecting Run as Administrator:

.. figure:: win_cmd_admin.png
   :alt: Launching an admin cmd

   Launching a Command Prompt with administrative rights

Now you just need to type the following commands:

.. code-block:: bash

  cd C:\MoorDyn\eigen.build
  mingw32-make
  mingw32-make install

We will need to use cmd with administrative rights later on, so do not close it.

Now we will install MoorDyn following a very similar process.
Launch CMake again, and set "C:\MoorDyn\MoorDyn" in the source box and
"C:\MoorDyn\MoorDyn.build" in the binaries box, clicking "Configure" afterwards.
Select again the "MinGW Makefiles" for the generator.
When the configuration options appear, set CMAKE_BUILD_TYPE as "Release", and
enable FORTRAN_WRAPPER and PYTHON_WRAPPER:

.. figure:: win_cmake_moordyn.png
   :alt: Configuration options for MoorDyn

   Configuration options for MoorDyn

You can also enable MATLAB_WRAPPER if you have Matlab installed in your system.
We are ready, click "Configure" once more and then "Generate".

Now go back to your Command Prompt from earlier (which has adminsitrative rights), and
type the following commands:

.. code-block:: bash

  cd C:\MoorDyn\MoorDyn.build
  mingw32-make
  mingw32-make install

NOTE: If you want to generate a Windows installer, disable the PYTHON_WRAPPER
option and type

.. code-block:: bash

  cd C:\MoorDyn\MoorDyn.build
  mingw32-make
  cpack -C Release


Linux and MAC
^^^^^^^^^^^^^

First of all, use your package manager to install the following packages

* `Git <https://git-scm.com/>`_
* `CMake <https://cmake.org/>`_
* `Python <https://www.python.org/>`_
* `Eigen3 <https://eigen.tuxfamily.org/>`_

In Linux you can use either `GCC <https://gcc.gnu.org/>`_ or
`CLang <https://clang.llvm.org/>`_, while in MAC the latter is the very only
option.
The process to compile and install is the same no matters the compiler you have
chosen.
However, it should be noticed that CLang does not provides a Fortran compiler.
To get Fortran support you would therefore install another compiler (e.g. the
GCC one)

In this tutorial we are assuming you have administrative rights in your system,
although it is also possible to install MoorDyn and the wrappers in the user
space.

First we are downloading the MoorDyn source code from the repository using git,

.. code-block:: bash

   cd $HOME
   git clone https://github.com/mattEhall/MoorDyn.git
   cd MoorDyn

Now we will ask cmake to configure everything typing

.. code-block:: bash

   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release ../

If for some reason you decided to do not install
`Eigen3 <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_ (although
it can be easily installed with your packages manager), you can still configure
MoorDyn adding the option -DEXTERNAL_EIGEN=OFF. Remember that in that case
you will only have available the :ref:`C API <api_c>`, not the
:ref:`C++ API <api_cpp>` one.

Finally you can compile and install MoorDyn:

.. code-block:: bash

   make -j
   make install

That will install the C and C++ headers in /usr/include/moordyn folder, the
library and the CMake configuration files (to allow other projects to easily
find and link it) in /usr/lib/ folder, and the Python wrapper in the appropriate
Python folder under /usr/lib/.

In case you do not have administrative priviledges, you can install MoorDyn
anywhere else just changing the option -DCMAKE_INSTALL_PREFIX=$HOME/.local while
configuring CMake. You also want to ask the Python wrapper get installed in the
user space with the option -DPYTHON_WRAPPER_USERINSTALL=ON.
You would need to edit the LD_LIBRARY_PATH environment variable afterwards.

If you have also installed the Fortran compiler, which is usually the case in
most Linux distributions, you can also compile and install the Fortran wrapper,
just setting the option -DFORTRAN_WRAPPER=ON.
Along the same line, if you have Matlab installed in your system, feel free to
add also the option -DMATLAB_WRAPPER=ON.

Use MoorDyn in your project
---------------------------

.. _starting_using:

The way you can use MoorDyn in your project depends of course on the language.
Below it is documented the way you can integrate MoorDyn in your project in
different languages. The details on the system definition file are provided
:ref:`here <usage>`, while the code is further documented
:ref:`here <coupling>`. If you have any problem try to give a look to the
:ref:`troubleshooting documentation <troubleshooting>`

C
^^^^^^

The easiest way to link MoorDyn to your C project is using CMake. Following
a code snippet where MoorDyn is integrated in a project with only a C source
code file named example.c:

.. code-block:: cmake

   cmake_minimum_required (VERSION 3.10)
   project (myproject)

   find_package (MoorDyn REQUIRED)

   add_executable (example example.c)
   target_link_libraries (example MoorDyn::moordyn)

CMake itself will already take care on everything. In the example.c you only
need to include the MoorDyn2.h header and start using the :ref:`C API <api_c>`,
as it is further discussed in the :ref:`coupling documentation <coupling>`.

.. code-block:: c

   #include <moordyn/MoorDyn2.h>

   int main(int, char**)
   {
      MoorDyn system = MoorDyn_Create("Mooring/lines.txt");
      MoorDyn_Close(system);
   }

C++
^^^^^^

The same CMake code snippet show above is equally valid for C++. In your C++
code you must remember start including the MoorDyn configuration header and then
the main header, i.e.

.. code-block:: cpp

   #include <moordyn/Config.h>
   #include <moordyn/MoorDyn2.hpp>

   int main(int, char**)
   {
      auto system = new moordyn::MoorDyn("Mooring/lines.txt");
      delete system;
   }

Python
^^^^^^

If you have installed the MoorDyn Python wrapper you are just ready to go! Open
a Python console and give it a shot!

.. code-block:: python

   import moordyn

   system = moordyn.Create("Mooring/lines.txt")
   moordyn.CLose(system)

Fortran
^^^^^^^

Linking the MoorDyn Fortran wrapper is almost the same than linking the C
library. For instance, if you have a Fortran project consisting in a single
source code file, example.f90, then you can integrate MoorDyn with the
following CMake code:

.. code-block:: cmake

   cmake_minimum_required (VERSION 3.10)
   project (myproject)

   find_package (MoorDyn REQUIRED)

   add_executable (example example.f90)
   target_link_libraries (example MoorDyn::moordynf)

Please, note that now you are linking against MoorDyn::moordynf. The usage
is also very similar to the C one:

.. code-block:: fortran

   program main
     use, intrinsic :: iso_c_binding, only: c_ptr
     use moordyn

     character(len=28) :: infile
     type(c_ptr) :: system
     integer :: err

     infile = 'Mooring/lines.txt'
     system = MD_Create(infile)
     err = MD_Close(system)

   end program main

Matlab
^^^^^^

Using MoorDyn in Matlab is so far similar to using it in Python. However, in
Matlab you must manually add the folder where the wrapper an MoorDyn libraries
are located to the path.
To this end, in Matlab go to the HOME menu, section ENVIRONMENT, and click on
"Set Path".
In the window appearing click on "Add Folder...", and set the folder where you
installed the MoorDyn library, which by default is:

* C:\Program Files (x86)\MoorDyn\bin in Windows
* /usr/lib in Linux and MacOS

After that you are free to go!

.. code-block:: matlab

   system = MoorDynM_Create("Mooring/lines.txt")
   MoorDynM_Close(system)

