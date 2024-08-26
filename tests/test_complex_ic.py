import sys
from unittest import TestCase, main as unittest_main
import os
import math
import moordyn


class ICTest(TestCase):
    def setUp(self):
        pass

    def test_no_input(self):
        try:
            system = moordyn.Create('file.invalid')
        except RuntimeError:
            return
        self.assertEqual(system, None, "The creation should fail")

    def test_one_iter(self):
        # 1st IC generation stage: The stationary solver
        system = moordyn.Create("Mooring/riser/riser_ic1.txt")
        point = moordyn.GetPoint(system, 4)
        r = moordyn.GetPointPos(point)
        v = [0, 0, 0]
        moordyn.Init(system, r, v)
        moordyn.SaveState(system, "Mooring/riser/riser.ic")
        moordyn.Close(system)
        # 2nd IC generation stage: The dynamic solver
        system = moordyn.Create("Mooring/riser/riser_ic2.txt")
        point = moordyn.GetPoint(system, 4)
        r = moordyn.GetPointPos(point)
        moordyn.Init(system, r, v)
        moordyn.SaveState(system, "Mooring/riser/riser.ic")
        moordyn.Close(system)
        # The simulation itself
        system = moordyn.Create("Mooring/riser/riser.txt")
        point = moordyn.GetPoint(system, 4)
        r = moordyn.GetPointPos(point)
        moordyn.Init(system, r, v)
        T = 10
        A = 0.0
        dt = T / 100
        t = 0.0
        step = 0
        while t <= 1 * T:
            rorg = moordyn.GetPointPos(point)
            rdst = list(rorg)
            rdst[2] = r[2] + A * math.sin(2 * math.pi * (t + dt) / T)
            v = [(rdst[i] - rorg[i]) / dt for i in range(len(rorg))]
            moordyn.Step(system, rorg, v, t, dt)
            try:
                moordyn.SaveVTK(system,
                                f"Mooring/riser/vtk/out.{step:05d}.vtm")
            except:
                pass
            t += dt
            step += 1
        moordyn.Close(system)


if __name__ == '__main__':
    unittest_main()
