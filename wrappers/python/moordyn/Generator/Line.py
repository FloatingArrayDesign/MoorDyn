from .Entity import Entity
from .Rod import Rod


class LineMaterial(Entity):
    """A line material
    """
    def __init__(self, name, d, w, ea, ba, ei, cdn, can, cdt, cat):
        """Constructor

        Parameters
        ----------
        name (str): The name of the line material
        d (float): The line diameter
        w (float): The weight per meter of line
        ea (float): The line stiffness, product of elasticity modulus and
                    cross-sectional area
        ba (float): The line internal damping (measured in N-s) or, if a
                    negative value is entered, the desired damping ratio (in
                    fraction of critical) for the line type
        ei (float): The bending stiffness
        cdn (float): transverse drag coefficient (with respect to frontal area, d*l)
        can (float): transverse added mass coefficient (with respect to line displacement)
        cdt (float): tangential drag coefficient (with respect to surface area, pi*d*l)
        cat (float): tangential added mass coefficient (with respect to line displacement)
        """

        Entity.__init__(
            self,
            "LINE TYPES",
            field_names=["Name", "Diam", "Mass/m", "EA", "BA/-zeta", "EI", "Cd", "Ca", "CdAx", "CaAx"],
            field_units=["(name)", "(m)", "(kg/m)", "(N)", "(N-s/-)", "(N-m^2)", "(-)", "(-)", "(-)", "(-)"])
        self.__name = name
        self.__d = d
        self.__w = w
        self.__ea = ea
        self.__ba = ba
        self.__ei = ei
        self.__cdn = cdn
        self.__can = can
        self.__cdt = cdt
        self.__cat = cat
        self.__set_values()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, n):
        self.__name = n
        self.__set_values()

    @property
    def d(self):
        return self.__d

    @d.setter
    def d(self, diam):
        self.__d = diam
        self.__set_values()

    @property
    def w(self):
        return self.__w

    @w.setter
    def w(self, weight):
        self.__w = weight
        self.__set_values()

    @property
    def ea(self):
        return self.__ea

    @ea.setter
    def ea(self, v):
        self.__ea = v
        self.__set_values()

    @property
    def ba(self):
        return self.__ba

    @ba.setter
    def ba(self, v):
        self.__ba = v
        self.__set_values()

    @property
    def ei(self):
        return self.__ei

    @ei.setter
    def ei(self, v):
        self.__ei = v
        self.__set_values()

    @property
    def cdn(self):
        return self.__cdn

    @cdn.setter
    def cdn(self, c):
        self.__cdn = c
        self.__set_values()

    @property
    def can(self):
        return self.__can

    @can.setter
    def can(self, c):
        self.__can = c
        self.__set_values()

    @property
    def cdt(self):
        return self.__cdt

    @cdt.setter
    def cdt(self, c):
        self.__cdt = c
        self.__set_values()

    @property
    def cat(self):
        return self.__cat

    @cat.setter
    def cat(self, c):
        self.__cat = c
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__name,
                         self.__d,
                         self.__w,
                         self.__ea,
                         self.__ba,
                         self.__ei,
                         self.__cdn,
                         self.__can,
                         self.__cdt,
                         self.__cat])


class LinePoint():
    """The line point
    """
    def __init__(self, obj, end_point="A"):
        self.__obj = obj
        self.__end_point = end_point  # Used just with Rods

    @property
    def obj(self):
        return self.__obj

    @obj.setter
    def obj(self, v):
        self.__obj = v

    @property
    def end_point(self):
        return self.__end_point

    @end_point.setter
    def end_point(self, v):
        self.__end_point = v

    def __str__(self):
        """Get the string

        Returns
        -------
        str: The point string
        """
        if isinstance(self.__obj, Rod):
            return "R" + str(self.__obj.name)
        return str(self.__obj.name)


class Line(Entity):
    """A line
    """
    def __init__(self, material, point0, point1, l, n):
        """Constructor

        Parameters
        ----------
        material (LineMaterial): The line material
        point0 (LinePoint): The line point
        point1 (LinePoint): The line point
        l (float): Unstretched length
        n (int): Number of segments
        """

        Entity.__init__(
            self,
            "LINES",
            field_names=["ID", "LineType", "AttachA", "AttachB", "UnstrLen", "NumSegs", "Outputs"],
            field_units=["(#)", "(name)", "(#)", "(#)", "(m)", "(-)", "(-)"])
        self.__name = 0
        self.__mat = material
        self.__point0 = point0
        self.__point1 = point1
        self.__l = l
        self.__n = n
        self.__set_values()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, n):
        self.__name = n
        self.__set_values()

    @property
    def material(self):
        return self.__mat

    @material.setter
    def material(self, material):
        self.__mat = material
        self.__set_values()

    @property
    def point0(self):
        return self.__point0

    @point0.setter
    def point0(self, point):
        self.__point0 = point
        self.__set_values()

    @property
    def point1(self):
        return self.__point1

    @point1.setter
    def point1(self, point):
        self.__point1 = point
        self.__set_values()

    @property
    def l(self):
        return self.__l

    @l.setter
    def l(self, v):
        self.__l = v
        self.__set_values()

    @property
    def n(self):
        return self.__n

    @n.setter
    def n(self,n):
        self.__n = n
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__name,
                         self.__mat.name,
                         self.__point0,
                         self.__point1,
                         self.__l,
                         self.__n,
                         "-"])
