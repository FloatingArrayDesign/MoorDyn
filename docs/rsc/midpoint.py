import numpy as np
import matplotlib.pyplot as plt


START_Y = 0.0
TARGET_Y = 1.0
C_0 = 0.0
C_1 = 0.075
OMEGA = 2. * np.pi
DT_INC = 0.1
DDRDDT_TOL = 1e-3


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


def ode2(iters, r0=1, backend=relax_all):
    dt_0 = dt = 2. / OMEGA * iters
    while True:
        r = r0
        ddrddt_0 = ddrddt = -OMEGA**2 * r0
        sol = -OMEGA**2 * (1. - 0.25 * OMEGA**2 * dt**2) * r0 / (1. + 0.25 * OMEGA**2 * dt**2)
        for i in range(iters):
            r = r0 + 0.25 * dt**2 * (ddrddt_0 + ddrddt)
            ddrddt_new = -OMEGA**2 * r
            if i < iters - 1:
                ddrddt = relax(ddrddt, ddrddt_new, i, iters, backend=backend)
            else:
                ddrddt = ddrddt_new
        if np.abs(ddrddt - sol) > DDRDDT_TOL:
            return dt - DT_INC * dt_0
        dt += DT_INC * dt_0
    return dt


def ode2_plot(N=range(2, 50, 1)):
    global C_0, C_1
    c0, c1 = C_0, C_1
    dts, c0s, c1s = [], [], []
    for n in N:
        print(n)
        best_dt, best_c0, best_c1 = 0.0, 0.0, 0.0
        # for C_0 in np.arange(0, 0.1, 0.001):
        #     for C_1 in np.arange(0.01, 0.5, 0.01):
        #         dt = ode2(n)
        #         if dt > best_dt:
        #             best_dt = dt
        #             best_c0 = C_0
        #             best_c1 = C_1
        # dts.append(best_dt)
        # c0s.append(best_c0)
        # c1s.append(best_c1)
        if n >= 14:
            C_0 = 0.01
            C_1 = 0.02
        else:
            C_0 = max(np.polyval([-0.011, 0.154], [n])[0], 0)
            C_1 = 0.08
        dt = ode2(n)
        dts.append(dt)
        c0s.append(C_0)
        c1s.append(C_1)
    dts = np.asarray(dts) * OMEGA / 2 / N
    plt.plot(N, dts, 'k-')
    plt.plot([N[0], N[-1]], [1.0, 1.0], 'k--')
    plt.xlabel(r'$M$')
    plt.ylabel(r'$\frac{\Delta t \omega}{2 N}$')
    plt.grid()
    plt.show()

    plt.plot(N, c0s, 'k-')
    plt.grid()
    plt.show()

    plt.plot(N, c1s, 'k-')
    plt.grid()
    plt.show()

    C_0, C_1 = c0, c1


if __name__ == '__main__':
    # oraculus_plot()
    ode2_plot()
