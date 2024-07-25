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


BODY_TYPE_COUPLED = POINT_TYPE_COUPLED = -1
BODY_TYPE_FREE = POINT_TYPE_FREE = 0
BODY_TYPE_FIXED = POINT_TYPE_FIXED = 1
ROD_TYPE_COUPLED = -2
ROD_TYPE_CPLDPIN = -1
ROD_TYPE_FREE = 0
ROD_TYPE_PINNED = 1
ROD_TYPE_FIXED = 2


ENDPOINT_A = ENDPOINT_BOTTOM = 0
ENDPOINT_B = ENDPOINT_TOP = 1


#                                  MoorDyn2.h
#  =============================================================================

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
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: The number of DOFs
    """
    import cmoordyn
    return cmoordyn.n_coupled_dof(instance)


def SetVerbosity(instance, verbosity):
    """Set the verbosity level of the MoorDyn instance

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    filepath (str): The log file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.set_logfile(instance, filepath)


def SetLogLevel(instance, verbosity):
    """Set the log file verbosity level of the MoorDyn instance

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    point positions specified by array x and their velocities in array v

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Keyword arguments:
    x (list): Position of the coupled points
    v (list): Velocity of the coupled points

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.init(instance, list(x), list(v), 0)


def Init_NoIC(instance, x, v):
    """Initializes the MoorDyn sytem, without calculating initial conditions

    Use this when you are planning to call moordyn.Load() afterwards

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Keyword arguments:
    x (list): Position of the coupled points
    v (list): Velocity of the coupled points

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.init(instance, list(x), list(v), 1)


def Step(instance, x, v, t, dt):
    """Compute a time step

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    x (list): Position of the coupled points
    v (list): Velocity of the coupled points
    t (float): The time instant
    dt (float): The time step

    Returns:
    list: The forces acting on the coupled points
    """
    import cmoordyn
    return cmoordyn.step(instance, list(x), list(v), t, dt)


def Close(instance):
    """This function deallocates the variables used by MoorDyn

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

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
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    cmoordyn.MoorDynWaves: The waves manager
    """
    import cmoordyn
    return cmoordyn.get_waves(instance)


def GetSeafloor(instance):
    """Get the 3D seafloor instance

    The is not None just if a SeafloorPath option was set in the input file.

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    cmoordyn.MoorDynSeafloor: The seafloor instance
    """
    import cmoordyn
    try:
        return cmoordyn.get_seafloor(instance)
    except RuntimeError:
        return None


def ExternalWaveKinInit(instance):
    """Initializes the external Wave kinetics

    This is useless unless WaveKin option is set to 1 in the input file. If
    that is the case, remember calling this function after moordyn.Init()

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.ext_wave_init(instance)


def ExternalWaveKinGetN(instance):
    """Get the number of points where the waves kinematics shall be provided

    This is useless unless WaveKin option is set to 1 in the input file.

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

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
    instance (cmoordyn.MoorDyn): The MoorDyn instance

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
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: The number of bodies
    """
    import cmoordyn
    return cmoordyn.get_number_bodies(instance)


def GetBody(instance, body):
    """ Get a rigid body

    Remember that the first body index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: The number of rods
    """
    import cmoordyn
    return cmoordyn.get_number_rods(instance)


def GetRod(instance, rod):
    """ Get a rod

    Remember that the first rod index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    rod (int): The rod index, starting at 1

    Returns:
    cmoordyn.MoorDynRod: The rod instance
    """
    import cmoordyn
    return cmoordyn.get_rod(instance, rod)


def GetNumberPoints(instance):
    """Get the number of points

    Remember that the first point index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: The number of points
    """
    import cmoordyn
    return cmoordyn.get_number_points(instance)


def GetPoint(instance, point):
    """ Get a point

    Remember that the first point index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    point (int): The point index, starting at 1

    Returns:
    cmoordyn.MoorDynPoint: The point instance
    """
    import cmoordyn
    return cmoordyn.get_point(instance, point)


def GetNumberLines(instance):
    """Get the number of mooring lines

    Remember that the first line index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    int: The number of mooring lines
    """
    import cmoordyn
    return cmoordyn.get_number_lines(instance)


def GetLine(instance, line):
    """ Get a mooring line

    Remember that the first line index is 1

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
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
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    n_lines (int): The number of lines

    Returns:
    list: The horizontal fairlead tension at each line
    list: The vertical fairlead tension at each line
    list: The horizontal anchor tension at each line
    list: The vertical anchor tension at each line
    """
    import cmoordyn
    data = cmoordyn.get_fast_tens(instance, n_lines)
    return data[0], data[1], data[2], data[3]


def Serialize(instance):
    """Serialize the MoorDyn system into a bytes array that can be restored
    afterwards to resume the simulation

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance

    Returns:
    Bytes array
    """
    import cmoordyn
    return cmoordyn.serialize(instance)


def Deserialize(instance, data):
    """Restores the MoorDyn system to a previous serialized state

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    data (bytes): The serialized data

    Returns:
    None
    """
    import cmoordyn
    cmoordyn.deserialize(instance, data)


def Save(instance, filepath):
    """Save the MoorDyn system into a file that can be loaded afterwards to
    resume the simulation

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    filepath (str): The file path

    Returns:
    None
    """
    import cmoordyn
    cmoordyn.save(instance, filepath)


def Load(instance, filepath):
    """Load a previously saved MoorDyn system, in order to resume the simulation

    You still must read the same definition file with moordyn.Create() and call
    moordyn.Init_NoIC()

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    filepath (str): The file path

    Returns:
    None
    """
    import cmoordyn
    cmoordyn.load(instance, filepath)


def SaveVTK(instance, filename):
    """ Save the rod to a VTK (.vtp) file

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.save_vtk(instance, filename)


#                                  Waves.h
#  =============================================================================


def GetWavesKin(instance, x, y, z, t, seafloor=None):
    """ Get the wave kinematics

    Parameters:
    instance (cmoordyn.MoorDynWaves): The waves instance
    x (float): The x coordinate
    y (float): The y coordinate
    z (float): The z coordinate
    t (float): The simulation time
    seafloor (cmoordyn.MoorDynSeafloor): The 3D seafloor instance

    Returns:
    u: The velocity (a tuple with 3 components)
    ud: The acceleration (a tuple with 3 components)
    zeta: The wave height
    pdyn: The dynamic pressure
    """
    import cmoordyn
    return cmoordyn.waves_getkin(instance, x, y, z, t, seafloor)


#                                  Seafloor.h
#  =============================================================================


def GetDepthAt(instance, x, y):
    """ Get the depth of the seafloor at some x and y

    Parameters:
    instance (cmoordyn.MoorDynSeafloor): The 3D seafloor instance
    x (float): The x coordinate
    y (float): The y coordinate

    Returns:
    depth: The output seafloor depth at that (x, y) point
    """
    import cmoordyn
    return cmoordyn.seafloor_getdepth(instance, x, y)
    

def GetAverageDepth(instance):
    """ Get the average of depth of the seafloor

    This value is calculated as the average value of every depth point

    If the rectilinear grid is not even, this average may not be the actual
    average depth of the surface the data describes.

    Parameters:
    instance (cmoordyn.MoorDynSeafloor): The 3D seafloor instance

    Returns:
    depth: The averaged seafloor depth
    """
    import cmoordyn
    return cmoordyn.seafloor_getavgdepth(instance)
    

def GetMinDepth(instance):
    """ The depth of the seafloor at the shallowest point

    Parameters:
    instance (cmoordyn.MoorDynSeafloor): The 3D seafloor instance

    Returns:
    depth: The minimum seafloor depth
    """
    import cmoordyn
    return cmoordyn.seafloor_getmindepth(instance)

#                                  Body.h
#  =============================================================================


def GetBodyID(instance):
    """ Get the body id

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    i: The body id
    """
    import cmoordyn
    return cmoordyn.body_get_id(instance)


def GetBodyType(instance):
    """ Get the body type

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    t: The body type
    """
    import cmoordyn
    return cmoordyn.body_get_type(instance)


def GetBodyState(instance):
    """ Get the body state

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    r: The position (a tuple with 6 components)
    u: The velocity (a tuple with 6 components)
    """
    import cmoordyn
    return cmoordyn.body_get_state(instance)


def GetBodyPos(instance):
    """ Get the body position

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    r: The position (a tuple with 3 components)
    """
    import cmoordyn
    return cmoordyn.body_get_pos(instance)


def GetBodyAngle(instance):
    """ Get the body Euler XYZ angles

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    r: The angles (a tuple with 3 components)
    """
    import cmoordyn
    return cmoordyn.body_get_angle(instance)


def GetBodyVel(instance):
    """ Get the body velocity

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    r: The velocity (a tuple with 3 components)
    """
    import cmoordyn
    return cmoordyn.body_get_vel(instance)


def GetBodyAngVel(instance):
    """ Get the body angular velocity

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    r: The angular velocity (a tuple with 3 components)
    """
    import cmoordyn
    return cmoordyn.body_get_angvel(instance)


def GetBodyForce(instance):
    """ Get the net force on the body

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    f: The force and the moment (a tuple with 6 components)
    """
    import cmoordyn
    return cmoordyn.body_get_force(instance)


def GetBodyM(instance):
    """ Get the mass matrix of the body

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance

    Returns:
    m: The mass matrix (a 6x6 components tuple)
    """
    import cmoordyn
    return cmoordyn.body_get_m(instance)


def SaveBodyVTK(instance, filename):
    """ Save the body to a VTK (.vtp) file

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.body_save_vtk(instance, filename)


def UseBodyVTK(instance, filename):
    """ Load the body representation from a 3d model file

    Parameters:
    instance (cmoordyn.MoorDynBody): The body instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.use_save_vtk(instance, filename)

#                                  Rod.h
#  =============================================================================


def GetRodID(instance):
    """ Get the rod id

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance

    Returns:
    i: The rod id
    """
    import cmoordyn
    return cmoordyn.rod_get_id(instance)


def GetRodType(instance):
    """ Get the rod type

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance

    Returns:
    i: The rod type
    """
    import cmoordyn
    return cmoordyn.rod_get_type(instance)


def GetRodForce(instance):
    """ Get the net force and moment acting over the rod

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance

    Returns:
    f: The rod force (a 6 components tuple)
    """
    import cmoordyn
    return cmoordyn.rod_get_force(instance)


def GetRodM(instance):
    """ Get the rod mass matrix

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance

    Returns:
    i: The rod mass matrix (a 6x6 components tuple)
    """
    import cmoordyn
    return cmoordyn.rod_get_m(instance)


def GetRodN(instance):
    """ Get the rod number of segments

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance

    Returns:
    n: The number of segments
    """
    import cmoordyn
    return cmoordyn.rod_get_n(instance)


def GetRodNodePos(instance, i):
    """ Get a rod node position

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance
    i (int): The node index

    Returns:
    r: The node position
    """
    import cmoordyn
    return cmoordyn.rod_get_node_pos(instance, i)


def GetRodNodeVel(instance, i):
    """ Get a rod node position

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance
    i (int): The node index

    Returns:
    r: The node velocity
    """
    import cmoordyn
    return cmoordyn.rod_get_node_vel(instance, i)


def SaveRodVTK(instance, filename):
    """ Save the rod to a VTK (.vtp) file

    Parameters:
    instance (cmoordyn.MoorDynRod): The rod instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.rod_save_vtk(instance, filename)

#                                Point.h
#  =============================================================================


def GetPointID(instance):
    """ Get the point id

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    i: The point id
    """
    import cmoordyn
    return cmoordyn.point_get_id(instance)


def GetPointType(instance):
    """ Get the point type

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    t: The point type
    """
    import cmoordyn
    return cmoordyn.point_get_type(instance)


def GetPointPos(instance):
    """ Get the point position

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    v: The point position
    """
    import cmoordyn
    return cmoordyn.point_get_pos(instance)


def GetPointVel(instance):
    """ Get the point velocity

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    v: The point velocity
    """
    import cmoordyn
    return cmoordyn.point_get_vel(instance)


def GetPointForce(instance):
    """ Get the point force

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    f: The point force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.point_get_force(instance)


def GetPointM(instance):
    """ Get the point mass matrix

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    m: The mass matrix (a 3x3 components tuple)
    """
    import cmoordyn
    return cmoordyn.point_get_m(instance)


def GetPointNAttached(instance):
    """ Get the number of attached lines

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance

    Returns:
    n: The number of attached lines
    """
    import cmoordyn
    return cmoordyn.point_get_nattached(instance)


def GetPointAttached(instance, i):
    """ Get an attached line

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance
    i (int): The index of the attachment

    Returns:
    l: The attached line
    e: The endpoint
    """
    import cmoordyn
    return cmoordyn.point_get_attached(instance, i)


def SavePointVTK(instance, filename):
    """ Save the point to a VTK (.vtp) file

    Parameters:
    instance (cmoordyn.MoorDynPoint): The point instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.point_save_vtk(instance, filename)

#                                  Line.h
#  =============================================================================


def GetLineID(instance):
    """ Get the line id

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    i: The line id
    """
    import cmoordyn
    return cmoordyn.line_get_id(instance)


def GetLineN(instance):
    """ Get the line number of segments

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    n: The number of segments
    """
    import cmoordyn
    return cmoordyn.line_get_n(instance)


def GetLineUnstretchedLength(instance):
    """ Get the line unstretched length

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    l: The unstreched length
    """
    import cmoordyn
    return cmoordyn.line_get_ulen(instance)


def SetLineUnstretchedLength(instance, l):
    """ Set the line unstretched length

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    l (float): The unstreched length
    """
    import cmoordyn
    return cmoordyn.line_set_ulen(instance, l)


def SetLineUnstretchedLengthVel(instance, v):
    """ Set the line unstretched length rate of change

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    v (float): The unstreched length rate of change
    """
    import cmoordyn
    return cmoordyn.line_set_ulenv(instance, v)


def IsLineConstantEA(instance):
    """ Get whether the line is governed by a non-linear stiffness or a
    constant one

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    b: True if the stiffness of the line is constant, False if a non-linear
       stiffness has been set
    """
    import cmoordyn
    return cmoordyn.line_is_const_ea(instance) != 0


def GetLineConstantEA(instance):
    """ Get the constant stiffness of the line
    
    This value is useless if non-linear stiffness is considered
    
    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    ea: The constant stiffness EA value
    """
    import cmoordyn
    return cmoordyn.line_get_const_ea(instance)


def SetLineConstantEA(instance, ea):
    """ Set the constant stiffness of the line
    
    This value is useless if non-linear stiffness is considered
    
    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    ea (float): The constant stiffness EA value
    """
    import cmoordyn
    return cmoordyn.line_set_const_ea(instance, ea)


def IsLinePressBend(instance):
    """ Get whether the line pressure bending is considered or not
    
    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    b: True if the pressure bending of the line is enabled, False otherwise
    """
    import cmoordyn
    return cmoordyn.line_is_pbend(b)


def SetLinePressBend(instance, b):
    """ Set whether the line pressure bending is considered or not
    
    This value is useless if non-linear stiffness is considered
    
    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    b (bool): True if the pressure bending of the line shall be considered,
              False otherwise
    """
    import cmoordyn
    return cmoordyn.line_set_pbend(instance, b)


def SetLinePressInt(instance, p):
    """Set the line internal pressure values at the nodes

    Parameters:
    instance (cmoordyn.MoorDyn): The MoorDyn instance
    p (list): Pressure values
    """
    import cmoordyn
    return cmoordyn.line_set_pint(instance, list(p))


def GetLineNodePos(instance, i):
    """ Get a line node position

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    r: The node position
    """
    import cmoordyn
    return cmoordyn.line_get_node_pos(instance, i)


def GetLineNodeVel(instance, i):
    """ Get a line node velocity

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    rd: The node velocity
    """
    import cmoordyn
    return cmoordyn.line_get_node_vel(instance, i)


def GetLineNodeForce(instance, i):
    """ Get a line node net force

    To get the components of the force use moordyn.GetLineNodeTen() ,
    moordyn.GetLineNodeBendStiff(), moordyn.GetLineNodeWeight() ,
    moordyn.GetLineNodeDrag() , moordyn.GetLineNodeFroudeKrilov() and
    moordyn.GetLineNodeSeaBedForce()

    Note that the net force is *NOT* the sum of all those components. For
    instance, the tension contribution on the internal nodes is the difference
    between the tensions of the adjacent segments, while
    moordyn.GetLineNodeTen() is returning the averaged value.

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_force(instance, i)


def GetLineNodeTen(instance, i):
    """ Get a line node tension

    This comprises the axial stiffness as well as the internal damping

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    t: The node tension (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_ten(instance, i)


def GetLineNodeBendStiff(instance, i):
    """ Get a line node bending stiffness force

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_bend(instance, i)


def GetLineNodeWeight(instance, i):
    """ Get a line node weight and bouyancy

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_w(instance, i)


def GetLineNodeDrag(instance, i):
    """ Get a line node drag force

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_drag(instance, i)


def GetLineNodeFroudeKrilov(instance, i):
    """ Get a line node Froude-Krilov force

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_froudekrylov(instance, i)


def GetLineNodeSeabedForce(instance, i):
    """ Get a line node seabed reaction

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    f: The node force (a 3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_froudekrylov(instance, i)


def GetLineNodeCurv(instance, i):
    """ Get a line node curvature

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    r: The node curvature
    """
    import cmoordyn
    return cmoordyn.line_get_node_curv(instance, i)


def GetLineNodeM(instance, i):
    """ Get a line node mass matrix

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    i (int): The node index

    Returns:
    r: The node mass matrix (a 3x3 components tuple)
    """
    import cmoordyn
    return cmoordyn.line_get_node_m(instance, i)


def GetLineFairTen(instance):
    """ Get the tension magnitude at the fairlead of a line

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    t: The tension magnitude
    """
    import cmoordyn
    return cmoordyn.line_get_fairlead_tension(instance)


def GetLineMaxTen(instance):
    """ Get the line maximum tension magnitude

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance

    Returns:
    t: The tension magnitude
    """
    import cmoordyn
    return cmoordyn.line_get_max_tension(instance)


def SaveLineVTK(instance, filename):
    """ Save the line to a VTK (.vtp) file

    Parameters:
    instance (cmoordyn.MoorDynLine): The line instance
    filepath (str): The file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return cmoordyn.line_save_vtk(instance, filename)
