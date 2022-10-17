"""moordyn is a Python wrapper for the MoorDyn v2 C library

It is used in the same way than the regular MoorDyn C API:

.. code-block:: python

    import moordyn
    system = moordyn.Create(filepath="Mooring/lines.txt")
    moordyn.SetVerbosity(system, moordyn.LEVEL_MSG)
    moordyn.SetLogFile(system, "Mooring/lines.log")
    moordyn.SetLogLevel(system, moordyn.LEVEL_MSG)
    moordyn.Log(system, "We are ready to work!")
    x = [5.2, 0.0, -70.0,
         -2.6, 4.5, -70.0,
         -2.6, -4.5, -70.0]
    v = [0, ] * 9
    moordyn.Init(system, x, v)
    dt = 0.1
    moordyn.Step(system, x, v, 0.0, dt)
    moordyn.Close(system)

The majority of functions are returning error codes, exactly the same way the
MoorDyn v2 C API is doing
"""

from .moordyn import *
from . import Generator
