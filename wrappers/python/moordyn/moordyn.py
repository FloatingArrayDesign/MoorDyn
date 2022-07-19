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
    MoorDyn: The MoorDyn instance
    """
    import cmoordyn
    return moordyn.create(filepath)


def NCoupledDOF(instance):
    """Get the number of coupled Degrees Of Freedom (DOF)

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    Returns:
    int: The number of DOFs
    """
    import cmoordyn
    return moordyn.n_coupled_dof(instance)


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
    return moordyn.set_verbosity(instance, verbosity)


def SetLogFile(instance, filepath):
    """Set the verbosity level of the MoorDyn instance

    Parameters:
    instance (MoorDyn): The MoorDyn instance
    filepath (str): The log file path

    Returns:
    int: 0 uppon success, an error code otherwise
    """
    import cmoordyn
    return moordyn.set_logfile(instance, filepath)


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
    return moordyn.set_loglevel(instance, verbosity)


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
    return moordyn.log(instance, level, head + msg)


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
    return moordyn.init(instance, list(x), list(v))


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
    return moordyn.step(instance, list(x), list(v), t, dt)


def Close(instance):
    """This function deallocates the variables used by MoorDyn

    Parameters:
    instance (MoorDyn): The MoorDyn instance

    """
    import cmoordyn
    return moordyn.close(instance)
