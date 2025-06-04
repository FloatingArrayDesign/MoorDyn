import moordyn
import numpy as np

'''
This is a very simple example of how to use the MoorDyn Python interface. Users need to provide a moordyn input file before running.
'''

# Create the MoorDyn system from the input file
system = moordyn.Create("<file name>")

# You can get the initial positions and velocities from the system itself using the MoorDyn-C API
body = moordyn.GetBody(system, 1)
state = moordyn.GetBodyState(body) # tuple with (x, xd)

x = np.array(state[0]) # x, y, z, roll, pitch, yaw 
xd = np.array(state[1]) # xdot, ydot, zdot, roll_dot, pitch_dot, yaw_dot 

# Setup the initial condition
moordyn.Init(system, x, xd)

# Set the simulation time
t, dt = 0.0, 0.5

# Run the simulation for five timesteps
for i in range(5):
    f = moordyn.Step(system, x, xd, t, dt)
    t += dt
    x += xd * dt # update the position 

# Alright, time to finish!
moordyn.Close(system)