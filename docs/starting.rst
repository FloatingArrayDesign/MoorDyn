Getting Started
===============

The latest version of MoorDyn v2 is now available on
`GitHub <https://github.com/mattEhall/moordyn/>`_. No binaries are provided
yet for MoorDyn v2, so you must compile it by yourself

Compile and install MoorDyn
---------------------------

In this tutorial it is assumed that you already have installed the following
softwares in your system:

* `Git <https://git-scm.com/>`_
* `CMake <https://cmake.org/>`_
* `Python <https://www.python.org/>`_

It is also required to install
`Eigen3 <https://eigen.tuxfamily.org/index.php?title=Main_Page>`_ if you are
planning to use the :ref:`C++ API <api_Cpp>` (For the :ref:`C API <api_C>` it is
not needed though).

The process is sligthly different in Windows than in the other platforms, so it
will be documented separatelly

Windows
^^^^^^^

On top of the softwares mentioned above, you must also install a compiler. You
can use one of the following:

* `MinGW <https://www.mingw-w64.org/>`_
* `Visual Studio <https://visualstudio.microsoft.com/>`_

MinGW is free and the community edition of Visual Studio cost no money.

Please, when you install each of those software, make sure you select the option
to add them to the PATH.

Finally, there is a `good tutorial to install Eigen3 <https://gist.github.com/danielTobon43/8ef3d15f84a43fb15f1f4a49de5fcc75>`_,
which again is strongly recommended.

WIP...

Linux and MAC
^^^^^^^^^^^^^

In Linux you can use either `GCC <https://gcc.gnu.org/>`_ or
`CLang <https://clang.llvm.org/>`_, while in MAC the latter is the very only
option. The process to compile and install is the same no matters the compiler
you have chosen.

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

Use MoorDyn
-----------

WIP
