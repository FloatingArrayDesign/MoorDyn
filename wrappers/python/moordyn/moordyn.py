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

LEVEL_NONE = 4096
LEVEL_ERR = 3
LEVEL_WRN = 2
LEVEL_MSG = 1
LEVEL_DBG = 0


ERRCODE_SUCCESS = 0
ERRCODE_INVALID_INPUT_FILE = -1
ERRCODE_INVALID_OUTPUT_FILE = -2
ERRCODE_INVALID_INPUT = -3
ERRCODE_NAN_ERROR = -4
ERRCODE_MEM_ERROR = -5
ERRCODE_INVALID_VALUE = -6
ERRCODE_NON_IMPLEMENTED = -7
ERRCODE_UNHANDLED_ERROR = -255



def Create(filepath=""):
    """Creates the MoorDyn system, loading the MoorDyn input file

    Keyword arguments:
    filepath (str): The file to load. If "" is passed, then "Mooring/lines.txt"
                    will be considered

    Returns:
    cmoordyn.MoorDyn: The MoorDyn instance
    """
    import cmoordyn
    return cmoordyn.create(filepath)


def NCoupledDOF(instance):
    """Get the number of coupled Degrees Of Freedom (DOF)

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of DOFs
    """
    import cmoordyn
    return cmoordyn.n_coupled_dof(instance)


def SetVerbosity(instance, verbosity):
    """Set the verbosity level of the MoorDyn instance

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    verbosity (int): The Verbosity level. One of:
                     - moordyn.LEVEL_NONE
                     - moordyn.LEVEL_ERR
                     - moordyn.LEVEL_WRN
                     - moordyn.LEVEL_MSG
                     - moordyn.LEVEL_DBG

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.set_verbosity(instance, verbosity)


def SetLogFile(instance, filepath):
    """Set the verbosity level of the MoorDyn instance

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    filepath (str): The log file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.set_logfile(instance, filepath)


def SetLogLevel(instance, verbosity):
    """Set the log file verbosity level of the MoorDyn instance

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    verbosity (int): The Verbosity level. One of:
                     - moordyn.LEVEL_NONE
                     - moordyn.LEVEL_ERR
                     - moordyn.LEVEL_WRN
                     - moordyn.LEVEL_MSG
                     - moordyn.LEVEL_DBG

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.set_loglevel(instance, verbosity)


def Log(instance, level, msg):
    """Set the log file verbosity level of the MoorDyn instance

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    level (int): The Verbosity level. One of:
                     - moordyn.LEVEL_NONE
                     - moordyn.LEVEL_ERR
                     - moordyn.LEVEL_WRN
                     - moordyn.LEVEL_MSG
                     - moordyn.LEVEL_DBG
    msg (str): The message to log

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    import inspect

    head = "DBG "
    if level >= LEVEL_ERR:
        head = "ERR "
    elif level == LEVEL_WRN:
        head = "WRN "
    elif level == LEVEL_MSG:
        head = "MSG "
    caller = inspect.stack()[1]
    head = head + "{}:{} {}() : ".format(caller.filename,
                                         caller.lineno,
                                         caller.function)
    return cmoordyn.log(instance, level, head + msg)


def Init(instance, x, v):
    """Initializes the MoorDyn sytem, calculating initial conditions based on
    connection positions specified by array x and their velocities in array v

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Keyword arguments:
    x (list): Position of the coupled connections
    v (list): Velocity of the coupled connections

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.init(instance, list(x), list(v))


def Step(instance, x, v, t, dt):
    """Compute a time step

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    x (list): Position of the coupled connections
    v (list): Velocity of the coupled connections
    t (float): The time instant
    dt (float): The time step

    Returns:
    list: The forces acting on the coupled connections
    """
    import cmoordyn
    return cmoordyn.step(instance, list(x), list(v), t, dt)


def Close(instance):
    """This function deallocates the variables used by MoorDyn

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.close(instance)


def GetWaves(instance):
    """Get the wave kinematics instance

    The wave kinematics instance is only useful if WaveKin option is set to >= 2
    in the input file.

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    cmoordyn.MoorDynWaves: The waves manager
    """
    import cmoordyn
    return cmoordyn.get_waves(instance)


def ExternalWaveKinInit(instance):
    """Initializes the external Wave kinetics

    This is useless unless WaveKin option is set to 1 in the input file. If
    that is the case, remember calling this function after moordyn.Init()

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.ext_wave_init(instance)


def ExternalWaveKinGetN(instance):
    """Get the number of points where the waves kinematics shall be provided

    This is useless unless WaveKin option is set to 1 in the input file.

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of points
    """
    import cmoordyn
    return cmoordyn.ext_wave_init(instance)


def GetWaveKinCoordinates(instance):
    """Get the points where the waves kinematics shall be provided

    The kinematics on those points shall be provided just if WaveKin is set
    to 1 in the input file

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    list (n, 3): The list of points
    """
    import cmoordyn

    coords = cmoordyn.ext_wave_coords(instance)
    n = len(coords) // 3
    points = []
    for i in range(n):
        points.append([coords[3 * i], coords[3 * i + 1], coords[3 * i + 2]])
    return points


def SetWaveKin(instance, u, a, t):
    """Set the kinematics of the waves

    Use this function if WaveKin option is set to 1 in the input file

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    u (list (n, 3)): The list of velocities evaluated in the points provided
                     by moordyn.GetWaveKinCoordinates()
    a (list (n, 3)): The list of accelerations evaluated in the points provided
                     by moordyn.GetWaveKinCoordinates()
    t (float): The simulation time

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn

    assert len(u) == len(a)
    uu, aa = [], []
    for i in range(len(u)):
        assert len(u[i]) == 3 and len(a[i]) == 3
        uu += u[i]
        aa += a[i]
    return cmoordyn.ext_wave_set(instance, uu, aa, t)


def GetNumberBodies(instance):
    """Get the number of bodies

    Remember that the first body index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of bodies
    """
    import cmoordyn
    return cmoordyn.get_number_bodies(instance)


def GetBody(instance, body):
    """ Get a rigid body

    Remember that the first body index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    body (int): The rigid body index, starting at 1

    Returns:
    cmoordyn.MoorDynBody: The rigid body instance
    """
    import cmoordyn
    return cmoordyn.get_body(instance, body)


def GetNumberRods(instance):
    """Get the number rods

    Remember that the rod index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of rods
    """
    import cmoordyn
    return cmoordyn.get_number_rods(instance)


def GetRod(instance, rod):
    """ Get a rod

    Remember that the first rod index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    rod (int): The rod index, starting at 1

    Returns:
    cmoordyn.MoorDynRod: The rod instance
    """
    import cmoordyn
    return cmoordyn.get_rod(instance, rod)


def GetNumberConnections(instance):
    """Get the number of connections

    Remember that the first connection index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of connections
    """
    import cmoordyn
    return cmoordyn.get_number_connections(instance)


def GetConnection(instance, conn):
    """ Get a connection

    Remember that the first connection index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    conn (int): The connection index, starting at 1

    Returns:
    cmoordyn.MoorDynConnection: The connection instance
    """
    import cmoordyn
    return cmoordyn.get_connection(instance, conn)


def GetNumberLines(instance):
    """Get the number of mooring lines

    Remember that the first line index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of mooring lines
    """
    import cmoordyn
    return cmoordyn.get_number_lines(instance)


def GetLine(instance, line):
    """ Get a mooring line

    Remember that the first line index is 1

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    line (int): The line index, starting at 1

    Returns:
    cmoordyn.MoorDynLine: The line instance
    """
    import cmoordyn
    return cmoordyn.get_line(instance, line)


def GetFASTtens(instance, n_lines):
    """Get the horizontal and vertical components of the fairlead and anchor
    tensions

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    n_lines (int): The number of lines

    Returns:
    list: The horizontal fairlead tension at each line
    list: The vertical fairlead tension at each line
    list: The horizontal anchor tension at each line
    list: The vertical anchor tension at each line
    """
    data = moordyn.get_fast_tens(instance, n_lines)
    return data[0], data[1], data[2], data[3]
