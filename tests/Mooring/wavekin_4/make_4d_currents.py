import numpy as np

T = 15.0
A = 1.0
UX = 1.25
x = np.linspace(-50.0, 50.0, num=11)
y = np.linspace(-2.0, 2.0, num=2)
z = np.linspace(-30.0, 5.0, num=36)
# t = np.linspace(0.0, 15.0, num=101)
t = np.linspace(0.0, 15.0, num=2)
zeta = A * np.sin(2.0 * np.pi * t / T)

# Define a current that increases in X dimension, pointing along the positive x-axis
# Doesn't vary current in any other dimension.
xx, yy, zz, tt = np.meshgrid(x, y, z, t)
ux = 0.1 * xx + 0.1 * t
uy = 0.0 * yy
uz = 0.05 * zz

with open('water_grid.txt', 'w') as f:
    f.write('--------------------- MoorDyn Waves grid File ----------------------------------\n')
    f.write('List of grid points, in 3 blocks (x, y, z)\n')
    f.write(
        'Each block starts with a 2 (i.e. list of coords), and then th coordinates (m)\n')
    for l in (x, y, z):
        txt = '1\n'
        for p in l:
            txt = txt + str(p) + ' '
        f.write(txt.strip() + '\n')

with open('current_profile_4d.txt', 'w') as f:
    # First line is number of (x,y,z,t) coords
    f.write(f"{len(x)} {len(y)} {len(z)} {len(t)}\n")

    # Next four lines specify the axis (tick) values for x, y, z, t respectively
    for xx in x:
        f.write(f"{xx} ")
    f.write("\n")

    for yy in y:
        f.write(f"{yy} ")
    f.write("\n")

    for zz in z:
        f.write(f"{zz} ")
    f.write("\n")

    for tt in t:
        f.write(f"{tt} ")
    f.write("\n")

    # All the remaining lines specify an (x, y, z, t) coordinate followed by
    # a (ux, uy, uz) current vector at that coordinate
    for idx, xs in enumerate(x):
        for idy, ys in enumerate(y):
            for idz, zs in enumerate(z):
                for idt, ts in enumerate(t):
                    f.write(
                        f"{xs} {ys} {zs} {ts} {ux[idy][idx][idz][idt]:.6g} {uy[idy][idx][idz][idt]} {uy[idy][idx][idz][idt]}\n")
