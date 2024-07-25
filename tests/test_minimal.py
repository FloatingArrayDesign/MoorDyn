import sys
from unittest import TestCase, main as unittest_main
import os
import tempfile
import moordyn


def setup_case(filename):
    """Setup a case in a temporal folder

    Parameters:
    filename (str): The filename in the "Mooring" subfolder

    Returns:
    str: The folder where the case has been generated
    """
    tmp_folder = tempfile.mkdtemp()
    # The input should be in the Mooring/lines.txt subfolder.
    # See MoorDyn.cpp:287
    input_folder = os.path.join(tmp_folder, "Mooring")
    os.mkdir(input_folder)
    r_path = os.path.join(
        os.path.dirname(__file__), "Mooring", filename)
    w_path = os.path.join(input_folder, "lines.txt")
    with open(r_path, "r") as r_file, open(w_path, "w") as w_file:
        w_file.write(r_file.read())
    return tmp_folder



class MinimalTests(TestCase):
    def setUp(self):
        pass

    def test_no_input(self):
        try:
            system = moordyn.Create('file.invalid')
        except RuntimeError:
            return
        self.assertEqual(system, None, "The creation should fail")

    def test_one_iter(self):
        tmp_folder = setup_case("lines.txt")
        cwd = os.getcwd()
        os.chdir(tmp_folder)
        system = moordyn.Create()
        os.chdir(cwd)
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
        print(forces)
        self.assertEqual(moordyn.Close(system),
                         0, "Failure finishing MoorDyn")
        f_ref = 7.2e5
        self.assertTrue(abs(forces[0] - f_ref) / f_ref < 0.01,
                        "Wrong surge force")


if __name__ == '__main__':
    unittest_main()
