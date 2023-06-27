.. _starting:

Getting Started
===============

MoorDyn is avaible in three versions: v1, v2, and F. V1 is the orginal MoorDyn code, containing just point and line objects. V2 is the upgraded
version of MoorDyn v1. It includes that capability to simulate rigid bodies, non-linear tension, 
wave kinematics, bending stiffness, and more. F is the fortran version of V2 implemented as a module in openFAST.
Futher details can be found in the :ref:`theory section <.. _theory>`. 

The latest version of MoorDyn v2 is now available on
`GitHub <https://github.com/mattEhall/moordyn/>`_.
This section describes both how to install MoorDyn
:ref:`using the python wrapper <_starting_binaries_python>`, how to :ref:`compile all features <_cmake_compile>`, 
and how to :ref:`compile a simple library <_library_compile>`. The compile instructions are broken down by operating system. 

.. Binaries and installers
.. -----------------------

.. .. _starting_binaries:

.. Two different cases must be considered when installing binaries, the
.. :ref:`C/C++ libraries <_starting_binaries_lib>` (with the Fortran wrapper), and
.. the :ref:`Python package <_starting_binaries_python>`.

.. C/C++ Library
.. ^^^^^^^^^^^^^

.. .. _starting_binaries_lib:

.. To install the C/C++ library (and the Fortran wrappers), please clone the V2 repository. Pre-compiled releases can be found at
.. `Releases page <https://github.com/mattEhall/MoorDyn/releases>`_. 
.. Along this line you would probably want to consider either the latest version
.. identified with a number, or the one named "nightly".
.. The former is the latest stable version, while the latter is the latest version
.. uploaded to the repository, which tends to be a bit less stable.

.. Once you already chosen a release, click on the assets and select the most
.. appropriate one for your platform.
.. More specifically, if you are in Windows you probably want to download and
.. execute Moordyn-X.Y.Z-win64.exe (with X.Y.Z replaced by the specific version),
.. in Linux you can download and execute Moordyn-X.Y.Z-Linux.sh and
.. in MacOS you can download and execute Moordyn-X.Y.Z-Darwin.sh.

.. NOTE: When you donwload the self-extracting files for Linux and MacOS they
.. cannot be launched until you give them execution permissions.

.. Now you can checkout
.. :ref:`how to integrate MoorDyn in your project <_starting_using>` below.

Compile and install MoorDyn - Python
------------------------------------

.. _starting_binaries_python:

MoorDyn V2 is avaible as a python module. To install, type

.. code-block:: bash

  python -m pip install moordyn

in your system terminal. Pip will take care of everything by you.

Compile and install MoorDyn - CMake
-----------------------------------

.. _cmake_compile:

The installation process is sligthly different depending on your operating system. Please see the corresponding section below:

Windows
^^^^^^^

In this tutorial we will install Eigen3 and MoorDyn in the default folders
(C:\Program Files (x86)\Eigen3 and C:\Program Files (x86)\Moordyn).
We will acquire the latest versions available with Git and build them
using CMake. This documentation assumes that you are building
in an MSYS2 build environment.

Install the following necessary tools:

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


Linux and Mac
^^^^^^^^^^^^^

First of all, use your package manager to install the following packages

* `Git <https://git-scm.com/>`_
* `CMake <https://cmake.org/>`_
* `Python <https://www.python.org/>`_
.. * `Eigen3 <https://eigen.tuxfamily.org/>`_

In Linux you can use either `GCC <https://gcc.gnu.org/>`_ or
`Clang <https://clang.llvm.org/>`_. On Mac OS, Clang is the only built-in 
compiler and any calls to GCC will be compiled with Clang. The process to 
compile and install is the same no matter which compiler you have installed.

However, it should be noted that Clang does not provide a Fortran compiler.
To get Fortran support you would install another compiler (e.g. the
GCC). 

In this tutorial we are assuming you have administrative rights in your system,
although it is also possible to install MoorDyn and the wrappers in the user
space.

First we download the MoorDyn source code from the repository using git,

.. code-block:: bash

   cd $HOME
   git clone https://github.com/mattEhall/MoorDyn.git
   cd MoorDyn

Now we will ask cmake to configure everything typing

.. code-block:: bash

   mkdir build
   cd build
   cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release ../

.. If for some reason you decided to do not install
.. `Eigen3 <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_ (although
.. it can be easily installed with your packages manager), you can still configure
.. MoorDyn adding the option -DEXTERNAL_EIGEN=OFF. Remember that in that case
.. you will only have available the :ref:`C API <api_c>`, not the
.. :ref:`C++ API <api_cpp>` one.

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

Compile and install MoorDyn - Simple Library
--------------------------------------------
.. _library_compile:

MoorDyn V2 can be compiled as a simple library that can be run in the driver file.
The installation location is dependent on your operating system. 

Before you begin, make sure the following tools are installed, along with a C++
compiler. On MacOS, the built in compiler is clang. 

* `Git <https://git-scm.com/>`_
* `CMake <https://cmake.org/>`_

Ensure you have the MoorDyn V2 source code installed. This can be done using git. 
Once MoorDyn is installed, change to the compile directory in terminal. The change to the directory 
corresponding to your operating system:

* `MoorDyn/compile/SO` for Linux
* `MoorDyn/compile/DYLIB` for MacOS
* `MoorDyn/compile/DLL` for Windows 

Once inside one of the three directories above, run the `make` command. MoorDyn 
will compile into a collection of object files (.o) and the library file (.so, .dylib, .dll).
This library file is what the driver function will call. 

Compile and install MoorDynF
----------------------------
Please refer to the `OpenFAST documentation <https://openfast.readthedocs.io/en/main/source/install/index.html>`_ for compile instructions for the MoorDynF module:

Note that is it possible to compile just the MoorDynF module using CMake, it is not necessary to compile all of OpenFAST.


Driving MoorDyn
---------------

.. _starting_using:

MoorDyn is avaible for use in your project in a variety of languages. It needs to be run
by a driver script written in one of the compatible languages: Python, C, C++, Fortran, and Matlab.
An example of python drivers can be found in the :ref:`examples section<_examples>`.
Any V2 driver function will call four MoorDyn functions:

- Create 
- Initialize
- Step
- Close

The intialize function takes the state vectors at time 0 and set up MoorDyn for a time series simulation.
The step function takes the state vectors (r - positions and rd - velocities) at a given time, the time, and the time step size. 
The step function needs to be called for each time step in your timeseries. 
The close function clears up memory. For both the step and the intialize functions, the input
state vector size needs to correspond to the DOF of the coupled object or objects. 
I.e. for 3 coupled 3DOF points, the vector size is 9 (3 * 3 = 9). Similarly 2 coupled 6DOF objects 
will have a vector size of 12. For example, the r vector for the 2 coupled 6DOF objects would be:

   [ x1, y1, z1, roll1, pitch1, yaw1, x2, y2, z2, roll2, pitch2, yaw2 ]

Input file details are provided
:ref:`here <usage>`, while the code is further documented
:ref:`here <coupling>`. 

If you have any problems, refer to:ref:`troubleshooting documentation <troubleshooting>`.

Python
^^^^^^

If you have installed the MoorDyn Python wrapper you are just ready to go! Open
a Python console and give it a shot!

.. code-block:: python

   import moordyn

   system = moordyn.Create("Mooring/lines.txt")
   moordyn.Close(system)

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

Fortran
^^^^^^^

This is not to be confused with MoorDynF, which relies on modules within the openFAST library.
MoorDynF when compiled includes a driver function with it's own driver input file. 
This coupling packages MoorDynV2 to be used in standalone Fortran projects. 
Linking the MoorDyn Fortran wrapper of MoorDynV2 is almost the same as linking the C
library. For instance, if you have a Fortran project consisting of a single
source code file, example.f90, then you can compile the driver with the
following CMake code:

.. code-block:: cmake

   cmake_minimum_required (VERSION 3.10)
   project (myproject)

   find_package (MoorDyn REQUIRED)

   add_executable (example example.f90)
   target_link_libraries (example MoorDyn::moordynf)

Please, note that now you are linking against MoorDyn::moordynf (not the same as MoorDynF). The usage
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

Using MoorDyn in Matlab is similar to using it in Python. However, in
Matlab you must manually add the folder where the wrapper and MoorDyn libraries
are located to the path.
To achieve this, in Matlab go to the HOME menu, section ENVIRONMENT, and click on
"Set Path".
In the window appearing click on "Add Folder...", and set the folder where you
installed the MoorDyn library, which by default is:

* C:\Program Files (x86)\MoorDyn\bin in Windows
* /usr/lib in Linux and MacOS

After that you are free to go!

.. code-block:: matlab

   system = MoorDynM_Create("Mooring/lines.txt")
   MoorDynM_Close(system)

Examples
^^^^^^^^
.. _examples:

The first example is driving MoorDyn from python using the python wrapper, assuming a stationary coupled body:

.. code-block:: python

   import moordyn
   import numpy as np

   rootname = 'lines'
   extension = '.txt'
   path = 'MooringTest/'
   tMax = 25.0
   dtM = 0.001
   time = np.arange(0, tMax, dtM)
   size = (len(time), 6)

   x = np.zeros(size)
   xd = np.zeros(size)

   system = moordyn.Create(path+rootname+extension)
   moordyn.Init(system, x[0,:], xd[0,:])
   # loop through coupling time steps
   print("MoorDyn initialized - now performing calls to MoorDynStep...")
   for i in range(1, len(time)):
      # call the MoorDyn step function
      print(time[i])
      moordyn.Step(system, x[i,:], xd[i,:], time[i], dtM)    #force value returned here in array

   print("Successfuly simulated for {} seconds - now closing MoorDyn...".format(tMax))  

   # close MoorDyn simulation (clean up the internal memory, hopefully) when finished
   moordyn.Close(system)   

The second example is driving MoorDyn from python using the MoorDynV1 API on MacOS, assuming a stationary coupled body:

.. code-block:: python

   import moordyn
   import ctypes
   import numpy as np

   rootname = 'lines_mod'
   extension = '.txt'
   path = 'MooringTest/'
   tMax = 25.0
   dtM = 0.001
   time = np.arange(0, tMax, dtM)
   vector_size = 6 # 6DOF coupled object
   size = (len(time), vector_size)

   x = np.zeros(size)
   xd = np.zeros(size)

   dylib_path = 'MoorDyn/compile/DYLIB/libmoordyn2.dylib'

   # -------------------- load the MoorDyn DYLIB ---------------------

   #Double vector pointer data type
   double_p = ctypes.POINTER(ctypes.c_double)
   vector_size = 6 # 6DOF coupled object

   # Make MoorDyn function prototypes and parameter lists (remember, first entry is return type, rest are args)
   MDInitProto = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), ctypes.c_char_p) #need to add filename option here, maybe this c_char works? #need to determine char size 
   MDStepProto = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), ctypes.POINTER(ctypes.c_double*vector_size), double_p, double_p)
   MDClosProto = ctypes.CFUNCTYPE(ctypes.c_int)

   MDInitParams = (1, "x"), (1, "xd"), (1, "infilename") # 1 flag is input, 2 flag is output
   MDStepParams = (1, "x"), (1, "xd"), (2, "f"), (1, "t"), (1, "dtC") 

   MDdylib = ctypes.CDLL(dylib_path) #load moordyn dylib

   filename = path+rootname+extension

   MDInit = MDInitProto(("MoorDynInit", MDdylib), MDInitParams)
   MDStep = MDStepProto(("MoorDynStep", MDdylib), MDStepParams)
   MDClose= MDClosProto(("MoorDynClose", MDdylib))  
   
   # ------------------------ run MoorDyn ---------------------------
   # initialize some arrays for communicating with MoorDyn
   t  = double_p()    # pointer to t

   # parameters
   dtC = ctypes.pointer(ctypes.c_double(dtM))
   infile = ctypes.c_char_p(bytes(filename, encoding='utf8'))

   # initialize MoorDyn at origin
   MDInit((x[0,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)),(xd[0,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)),infile)
   print("MoorDyn initialized - now performing calls to MoorDynStep...")

   # loop through coupling time steps
   for i in range(len(time)):
      t = ctypes.pointer(ctypes.c_double(time[i]))
      MDStep((x[i,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)), (xd[i,:]).ctypes.data_as(ctypes.POINTER(ctypes.c_double*vector_size)), t, dtC)    
   print("Succesffuly simulated for {} seconds - now closing MoorDyn...".format(tMax))  

   # close MoorDyn simulation (clean up the internal memory, hopefully) when finished
   MDClose() 

Notes on the Python V1 API:

- Using the V1 API does not call the create function because the V1 API does not allow for simulatneous MoorDyn instances. 
- The initalize function is MDInit.   
- MoorDyn functions require C++ data types as inputs
