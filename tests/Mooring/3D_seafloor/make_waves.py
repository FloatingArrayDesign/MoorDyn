import numpy as np

T = 15.0
A = 5.0
UX = 1.25
x = np.linspace(-450.0, 50.0, num=501)
y = np.linspace(-2.0, 2.0, num=5)
z = np.linspace(-51.0, 5.0, num=57)
t = np.linspace(0.0, 15.0, num=101)
zeta = A * np.sin(4.0 * 2.0 * np.pi * t / T)

with open('water_grid.txt', 'w') as f:
    f.write('--------------------- MoorDyn Waves grid File ----------------------------------\n')
    f.write('List of grid points, in 3 blocks (x, y, z)\n')
    f.write('Each block starts with a 2 (i.e. list of coords), and then th coordinates (m)\n')
    for l in (x, y, z):
        txt = '1\n'
        for p in l:
            txt = txt + str(p) + ' '
        f.write(txt.strip() + '\n')

with open('wave_elevation.txt', 'w') as f:
    for i, tt in enumerate(t):
        f.write(str(tt) + ' ' + str(zeta[i]) + '\n')

with open('current_profile.txt', 'w') as f:
    f.write('--------------------- MoorDyn steady currents File ----------------------------------\n')
    f.write('Tabulated file with the water currents components\n')
    f.write('z (m), ux (m/s), uy (m/s), uz (m/s)\n')
    for zz in z:
        ux = 0.0
        if zz > -50.0 and zz <= -10.0:
            ux = UX * (1.0 - (-zz - 10.0) / 40.0)
        elif zz > -10.0 and zz < -A:
            ux = UX * (1.0 - (zz + 10.0) / (10.0 - A))
        f.write(str(zz) + " " + str(ux) + " 0 0\n")
