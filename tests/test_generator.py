import sys
from unittest import TestCase, main as unittest_main
import moordyn


class GeneratorTests(TestCase):
    def setUp(self):
        pass

    def test_generation(self):
        system = moordyn.Generator.Mooring()
        line_type = moordyn.Generator.LineMaterial(
            "main", 0.09, 77.7066, 384.243E6, -0.8, 0, 1.6, 1.0, 0.1, 0.0)
        system.AddLineMaterial(line_type)

        points = [
            moordyn.Generator.Point(
                "Fixed", [853.87, 0, -320.0], 0, 0, 0, 0),
            moordyn.Generator.Point(
                "Fixed", [-426.94, 739.47, -320.0], 0, 0, 0, 0),
            moordyn.Generator.Point(
                "Fixed", [-426.94, -739.47, -320.0], 0, 0, 0, 0),
            moordyn.Generator.Point(
                "Coupled", [5.2, 0, -70.0], 0, 0, 0, 0),
            moordyn.Generator.Point(
                "Coupled", [-2.6, 4.5, -70.0], 0, 0, 0, 0),
            moordyn.Generator.Point(
                "Coupled", [-2.6, -4.5, -70.0], 0, 0, 0, 0),
        ]
        for point in points:
            system.AddPoint(point)

        lines = []
        for i in range(3):
            lines.append(moordyn.Generator.Line(
                line_type,
                moordyn.Generator.LinePoint(points[i]),
                moordyn.Generator.LinePoint(points[i + 3]),
                902.2,
                20
            ))
        for line in lines:
            system.AddLine(line)

        system.AddOption(moordyn.Generator.Option("writeLog", 2))
        system.AddOption(moordyn.Generator.Option("dtM", 0.002))
        system.AddOption(moordyn.Generator.Option("kBot", 3.0e6))
        system.AddOption(moordyn.Generator.Option("cBot", 3.0e5))
        system.AddOption(moordyn.Generator.Option("WtrDnsty", 1025.0))
        system.AddOption(moordyn.Generator.Option("WtrDpth", 320))
        system.AddOption(moordyn.Generator.Option("dtIC", 1.0))
        system.AddOption(moordyn.Generator.Option("TmaxIC", 100.0))
        system.AddOption(moordyn.Generator.Option("CdScaleIC", 4.0))
        system.AddOption(moordyn.Generator.Option("threshIC", 0.001))

        system = system.Create()

        x = []
        for i in range(4, 7):
            point = moordyn.GetPoint(system, i)
            x = x + list(moordyn.GetPointPos(point))
        v = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.assertEqual(moordyn.Init(system, x, v), 0,
                         "Failure initializing the lines")
        self.assertEqual(moordyn.NCoupledDOF(system), 9,
                         "Wrong number of coupled DOFs")
        self.assertEqual(moordyn.GetNumberLines(system), 3)
        v[0] = 0.1
        forces = moordyn.Step(system, x, v, 0.0, 0.5)
        self.assertEqual(moordyn.Close(system),
                         0, "Failure finishing MoorDyn")


if __name__ == '__main__':
    unittest_main()
