import moordyn
import numpy as np

### OPTIONAL ###
# import matplotlib.pyplot as plt
# import moorpy 

'''
This is a very simple example of how to use the MoorDyn Python interface. Users need to provide a moordyn input file before running.
This code is ready to run if a user provides an input file with a single coupled body. A more thorough explanation of how to drive 
MoorDyn can be found in `MoorDyn_standalone_demo.ipynb`.

The framework here is set up for a single coupled body. For other configurations, the state (x) and state derivatives (xd) need to be 
changed accordingly.
'''

def sin (period = 150, A = 10, axis = 0, dof = 6, x_initial = np.array([0, 0, 0, 0, 0, 0]), vector_size = 6, time = np.array([0])):
    '''
    A function that provides time series of states and state variables for simulating sinusoidal motion of a 6 DOF body in one of 
    the DOFs.

    Inputs
    ------
    `period` : `float`
        The period of the sinusoidal motion.
    `A` : `float`
        The amplitude of the sinusoidal motion.
    `axis` : `int`
        The axis along which the motion occurs (0 -> x, 1 -> y, 2 -> z, 3 -> roll, 4 -> pitch, 5 -> yaw).
    `dof` : `int`
        The degrees of freedom of the system.
    `x_initial` : `np.array`
        The initial state of the system.
    `vector_size` : `int`
        The size of the state vector. This should be equal to `DOF * the number of objects`. For example, 3 coupled points would have `DOF = 3` and `vector_size = 9`.
    `time` : `np.array`
        The time array for the simulation.

    Outputs
    -------
    `xp` : `np.array`
        The position time series of the system.
    `xdp` : `np.array`
        The velocity time series of the system.
    '''

    # axis 0 -> x, 1 -> y, 2 -> z, 3 -> roll, 4 -> pitch, 5 -> yaw
    xp = np.zeros((len(time),6))
    
    # Time info
    dt = time[1] - time[0]

    # Wave properties
    T = period / dt
    omega = (2*np.pi)/T
    
    for i in range(len(time)):
        xp[i,axis] = A * np.sin(i*omega)

    xdp = np.zeros((len(time),6))
    xold = np.zeros(6)
    # calculate velocities using finite difference
    for i in range(len(time)):
        xdp [i] = (xp[i] - xold)/dt
        xold =  xp[i]
    for i in range(len(time)):
        if i == 0:
            x[i,:] = x_initial
        else:
            j = 0
            while j < vector_size:
                x[i,j:j+dof] = x[i-1,j:j+dof] + xdp[i, 0:dof] * dt
                xd[i,j:j+dof] = xdp[i, 0:dof]
                j += dof

    return x, xd

# Create time array
tMax = 0.5 # max time for running time series
dt = 0.001 # coupling timestep
time = np.arange(0, tMax, dt) # time series

# Build arrays of state and state derivatives
size = (len(time), 6)
x = np.zeros(size)
xd = np.zeros(size)

# Create the MoorDyn system from the input file. 
# NOTE TO USERS:
#   Change this to your own input file. 
#   It must have one coupled body and no other coupled objects. 
system = moordyn.Create("<file name>")

# You can get the initial positions and velocities from the system itself using the MoorDyn-C API
body = moordyn.GetBody(system, 1)
state = moordyn.GetBodyState(body) # tuple with (x, xd)

# Set the first time step of the state and state derivatives to the initial values from MoorDyn
x[0,:] = np.array(state[0]) # x, y, z, roll, pitch, yaw
xd[0,:] = np.array(state[1]) # xdot, ydot, zdot, roll_dot, pitch_dot, yaw_dot

# Setup the initial condition
moordyn.Init(system, x[0,:], xd[0,:])

# Get the state and state derivative time series
x_t, xd_t = sin(period = 5, A = 1, axis = 0, dof = 6, x_initial = x[0,:], vector_size = 6, time = time)

# Run the simulation for five timesteps
for i in range(1, len(time)):
    # call the MoorDyn step function
    f = moordyn.Step(system, x_t[i,:], xd_t[i,:], time[i], dt)    # force value on coupled DOF returned here in array

# Alright, time to finish!
moordyn.Close(system)


### Optional ###

# # Plot and animate the results using MoorPy

# # Create a MoorPy system (this loads the moordyn outputs found in <dirname>/<rootname>_<object>#.out, where <object> is either Line or Rod and # is the corresponding number)
# ms = moorpy.System(file=<path+rootname+extension>, dirname=<path>, rootname=<rootname>, qs = 0, Fortran = False) # qs tells MoorPy it is loading a MoorDyn system, Fortran = False tells it to use the MoorDyn-C output format

# # Plot the results at t = 0 in red
# ms.plot(color="red", time = 0)

# # animate the results
# animation = ms.animateLines()

# plt.show()