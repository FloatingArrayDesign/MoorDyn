import os
import tempfile
from .Intro import Intro
from .. import moordyn


INPUT_FNAME = "mooring.txt"


class Mooring:
    """A mooring system that can be programmatically generated

    The target of this class is not checking that the provided inputs are
    correct. MoorDyn itself will take care of that.
    """
    def __init__(self):
        # Generate the temporal folder where we are working
        self.__root_folder = tempfile.mkdtemp()
        self.__input_file_path = os.path.join(self.__root_folder, "moordyn.txt")
        # The lists of objects
        self.__intro = Intro()
        self.__line_materials = []
        self.__rod_materials = []
        self.__bodies = []
        self.__rods = []
        self.__points = []
        self.__lines = []
        self.__failures = []
        self.__options = []
        # The MoorDyn instance
        self.__instance = None

    def AddLineMaterial(self, mat):
        self.__line_materials.append(mat)

    def AddRodMaterial(self, mat):
        self.__rod_materials.append(mat)

    def AddBody(self, body):
        self.__bodies.append(body)
        body.name = len(self.__bodies)

    def AddRod(self, rod):
        self.__rods.append(rod)
        rod.name = len(self.__rods)

    def AddPoint(self, point):
        self.__points.append(point)
        point.name = len(self.__points)

    def AddLine(self, line):
        self.__lines.append(line)
        line.name = len(self.__lines)

    def AddFailure(self, failure):
        self.__failures.append(failure)

    def AddOption(self, option):
        self.__options.append(option)

    def __write_input_file(self):
        with open(self.__input_file_path, "w") as f:
            f.write(self.__intro.get_header())
            self.__write_section(f, self.__line_materials)
            self.__write_section(f, self.__rod_materials)
            self.__write_section(f, self.__bodies)
            self.__write_section(f, self.__rods)
            self.__write_section(f, self.__points)
            self.__write_section(f, self.__lines)
            self.__write_section(f, self.__failures)
            self.__write_section(f, self.__options)
            f.write(self.__get_footer())

    def __write_section(self, f, lst):
        if not len(lst):
            return
        w = lst[0].get_field_widths()
        for v in lst:
            v.set_field_widths(w)
            w = v.get_field_widths()
        for v in lst:
            v.set_field_widths(w)
        hdr = False
        for v in lst:
            if not hdr:
                f.write(v.get_header())
                hdr = True
            f.write(v.get_values())

    def __get_footer(self):
        return ("-" * 80) + "\n"

    def Create(self):
        self.__write_input_file()
        print("Created the MoorDyn file: '{}'".format(self.__input_file_path))
        print("")
        print("=" * 80)
        print("")
        with open(self.__input_file_path, "r") as f:
            print(f.read())
        print("")
        print("=" * 80)
        print("")
        return moordyn.Create(filepath=self.__input_file_path)
