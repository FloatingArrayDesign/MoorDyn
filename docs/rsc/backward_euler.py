import numpy as np
import matplotlib.pyplot as plt


START_Y = 0.0
TARGET_Y = 1.0
C_0 = 0.0
C_1 = 0.075
OMEGA = 2. * np.pi
DT_INC = 0.1
DDRDDT_TOL = 1e-6


def relax_constant(i, n):
    return C_0 / n


def relax_tanh(i, n):
    x = (i + 1) / n
    return C_1 * (1 / n) * np.tanh(x)


def relax_bell(i, n):
    x = (i + 1) / n
    if x > 0.5:
        x = 1. - x
    return C_1 * (1 / n) * np.tanh(x)


def relax_all(i, n):
    x = (i + 1) / n
    return relax_constant(i, n) + relax_tanh(i, n)


def relax(y0, y1, i, n, backend=relax_all):
    f = backend(i, n)
    return (1. - f) * y0 + f * y1


def oraculus(iters, backend=relax_all):
    x = np.arange(0, iters + 1)
    y = [START_Y, ]
    f = [0.0, ]
    for i in range(iters - 1):
        y.append(relax(y[-1], TARGET_Y, i, iters, backend=backend))
        f.append(backend(i, iters))
    y.append(TARGET_Y)
    f.append(1.0)
    return x, np.asarray(y), np.asarray(f)


def oraculus_plot(N=(10, 100, 1000)):
    fig, (ax1, ax2) = plt.subplots(2, 1)
    for n, c in zip(N, ('k', 'r', 'b')):
        x, y, f = oraculus(n)
        ax1.plot(x / n, y, color=c,
                 label=f'M = {n}, C_0 = {C_0}, C_1 = {C_1}')
        ax2.plot(x[1:-1] / n, 1 - f[1:-1], color=c,
                    label=f'M = {n}, C_0 = {C_0}, C_1 = {C_1}')

    ax1.set_ylabel(r'$\mathrm{d} r / \mathrm{d} t$')
    ax1.set_xlim(0.0, 1.0)
    ax1.set_ylim(0.0, 1.0)
    ax2.set_xlabel(r'$m / M$')
    ax2.set_ylabel('$1 - f$')
    ax2.set_xlim(0.0, 1.0)
    ax1.grid()
    ax2.grid()
    ax1.legend(loc='best')
    plt.show()
    

def ode2(iters, r0=1, backend=relax_all):
    dt_0 = dt = 2. / OMEGA * iters
    while True:
        r = r0
        ddrddt = -OMEGA**2 * r
        sol = -OMEGA**2 * r / (1. + 0.5 * OMEGA**2 * dt**2)
        for i in range(iters):
            r = r0 + 0.5 * dt**2 * ddrddt
            ddrddt_new = -OMEGA**2 * r
            if i < iters - 1:
                ddrddt = relax(ddrddt, ddrddt_new, i, iters, backend=backend)
            else:
                ddrddt = ddrddt_new
            if np.abs(ddrddt - sol) < DDRDDT_TOL:
                break
        if ddrddt > 0 or -0.5 * dt**2 * ddrddt > 2. * r0:
            return dt - DT_INC * dt_0
        dt += DT_INC * dt_0
    return dt


def ode2_plot(N=range(5, 101, 2)):
    global C_0, C_1
    c0, c1 = C_0, C_1
    dts = []
    for n in N:
        C_0 = max(np.polyval([-0.01, 0.1], [n])[0], 0)
        if n >= 10:
            C_1 = max(1 / np.polyval([0.051, 0, 10.0], [n])[0], 0)
        else:
            C_1 = 0.07
        dt = ode2(n)
        dts.append(dt)
    dts = np.asarray(dts) * OMEGA / 2 / N
    plt.plot(N, dts, 'k-')
    plt.plot([N[0], N[-1]], [1.0, 1.0], 'k--')
    plt.xlabel(r'$M$')
    plt.ylabel(r'$\frac{\Delta t \omega}{2 N}$')
    plt.grid()
    # for x, y, c0p, c1p in zip(N, dts, c0s, c1s):
    #     plt.text(x, y, f'{c0p:.3f},{c1p:.3f}')
    plt.show()
    C_0, C_1 = c0, c1


if __name__ == '__main__':
    # oraculus_plot()
    ode2_plot()
