.. _troubleshooting:

Troubleshooting
===============

The Python wrapper does not work
--------------------------------

ModuleNotFoundError: No module named 'moordyn'
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you type in a Python console

.. code-block:: python

   import moordyn

and you receive the error above, then the Python wrapper is not properly
installed. First, if you compiled MoorDyn by yourself check that
PYTHON_WRAPPER option was ON while configuring with CMake.

Another possible source for the error is that MoorDyn was installed for a
different version of Python. That would happened if either CMake considered the
wrong version or you upgraded your Python installation. Either way, it is
recommended to install the MoorDyn Python wrapper again.

ImportError: libmoordyn.so.2: cannot open shared object file: No such file or directory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you type in a Python console

.. code-block:: python

   import moordyn
   moordyn.Create()

receiving the error above, then the Python wrapper has been correctly installed,
but the actual MoorDyn library cannot be found. This is generally caused by an
installation of MoorDyn in a custom path. You must locate where you have
installed the MoorDyn library and add that folder to the following environment
variables:

* LD_LIBRARY_PATH in Linux and MAC
* PATH in Windows

Note that the error can be slightly different in Linux, Windows and MAC.
