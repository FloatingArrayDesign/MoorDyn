import numpy as np
BASE_DEPTH = -30
x = np.linspace(-50.0, 50.0, num=100)
y = np.linspace(-2.0, 2.0, num=5)

# Define 2d grid for seafloor:
xx, yy = np.meshgrid(x,y)

with open('seafloor_profile_3d.txt', 'w') as f:
    # First line is number of (x,y) coords
    f.write(f"{len(x)} {len(y)}\n")
    
    # Next two lines specify the axis (tick) values for x, y respectively
    for xx in x:
        f.write(f"{xx} ")
    f.write("\n")

    for yy in y:
        f.write(f"{yy} ")
    f.write("\n")

    # All the remaining lines specify an (x, y, z) coordinate
    for idx, xs in enumerate(x):
        for idy, ys in enumerate(y):
            f.write(f"{xs} {ys} {BASE_DEPTH + 0.1 * xs}\n")
