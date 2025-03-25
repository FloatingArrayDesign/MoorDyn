from unittest import TestCase, main as unittest_main
import os
import csv
import math
import moordyn
from test_minimal import setup_case


MBL = 25.3e6
KRS = 13.4
KRD1 = 16.0
KRD2 = 0.35
TC = 200.0
TTIMES = [320.0, 640.0, 960.0, 1280.0, 1599.0]
TMEANS = [0.1, 0.2, 0.3, 0.4, 0.5]


def vec_norm(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def get_average_tension(line, anchor, fairlead):
    n = moordyn.GetLineN(line)
    # To average the tensions along the line, we are considering the
    # connections instead of the line end up nodes, as long as the latter are
    # not that accurate
    tension = 0.0
    # force = moordyn.GetPointForce(anchor)
    # tension += vec_norm(force)
    force = moordyn.GetPointForce(fairlead)
    tension += vec_norm(force)
    for i in range(n):
        force = moordyn.GetLineNodeTen(line, i)
        tension += vec_norm(force)
    return tension / (n + 1)


class PolyesterTests(TestCase):
    def setUp(self):
        pass

    def test_rampup_stabilization_cycles(self):
        global TTIMES, TMEANS

        tmp_folder = setup_case("polyester/simple.txt")
        cwd = os.getcwd()
        os.chdir(tmp_folder)
        system = moordyn.Create()
        os.chdir(cwd)

        n_dof = moordyn.NCoupledDOF(system)
        self.assertEqual(moordyn.NCoupledDOF(system), 3,
                         "Incorrect number of DOFs")

        anchor = moordyn.GetPoint(system, 1)
        fairlead = moordyn.GetPoint(system, 2)
        line = moordyn.GetLine(system, 1)

        l0 = moordyn.GetLineUnstretchedLength(line)
        r = moordyn.GetPointPos(fairlead)
        dr = [0, 0, 0]
        moordyn.Init(system, r, dr)

        tdata, xdata = [], []
        with open('Mooring/polyester/motion.csv') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='"')
            skip = 2
            for row in reader:
                if skip > 0:
                    skip -= 1
                    continue
                tdata.append(float(row[0]))
                xdata.append(float(row[1]))

        t = 0
        times, tensions = [], []
        for i in range(len(tdata)):
            t_dst = tdata[i]
            x_dst = xdata[i]
            dt = t_dst - t
            dr[0] = (x_dst - r[0]) / dt
            f = moordyn.Step(system, r, dr, t, dt)
            t += dt
            times.append(t)
            tensions.append(get_average_tension(line, anchor, fairlead))

            if (times[-1] - times[0] < TC):
                continue

            tension = 0.0;
            for f in tensions:
                tension += f
            tension /= len(tensions)
            times = times[1:]
            tensions = tensions[1:]

            ks = KRS * MBL
            kd = (KRD1 + KRD2 * tension / MBL * 100) * MBL
            l = l0 * (1.0 + tension / ks) / (1.0 + tension / kd)
            moordyn.SetLineConstantEA(line, kd)
            moordyn.SetLineUnstretchedLength(line, l)

            if (t >= TTIMES[0]):
                tmean = TMEANS[0]
                TTIMES = TTIMES[1:]
                TMEANS = TMEANS[1:]
                assert(abs(tension / MBL - tmean) < 0.025)

        moordyn.Close(system)

if __name__ == '__main__':
    unittest_main()
