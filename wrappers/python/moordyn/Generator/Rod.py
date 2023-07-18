from .Entity import Entity, PropsList


class RodMaterial(Entity):
    """A rod material
    """
    def __init__(self, name, d, w, cdn, can, cdt, cat):
        """Constructor

        Parameters
        ----------
        name (str): The name of the rod material
        d (float): The rod diameter
        w (float): The weight per meter of rod
        cdn (float): transverse drag coefficient (with respect to frontal area, d*l)
        can (float): transverse added mass coefficient (with respect to line displacement)
        cdt (float): tangential drag coefficient (with respect to surface area, pi*d*l)
        cat (float): tangential added mass coefficient (with respect to line displacement)
        """

        Entity.__init__(
            self,
            "ROD TYPES",
            field_names=["Name", "Diam", "Mass/m", "Cd", "Ca", "CdAx", "CaAx"],
            field_units=["(name)", "(m)", "(kg/m)", "(-)", "(-)", "(-)", "(-)"])
        self.__name = name
        self.__d = d
        self.__w = w
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
                         self.__cdn,
                         self.__can,
                         self.__cdt,
                         self.__cat])


class RodPoint():
    """The rod point
    """
    def __init__(self):
        self.__fixed = True
        self.__pinned = False
        self.__coupled = False
        self.__body = None

    @property
    def fixed(self):
        return self.__fixed

    @fixed.setter
    def fixed(self, v):
        self.__fixed = v
        if v:
            self.__coupled = False

    @property
    def pinned(self):
        return self.__pinned

    @pinned.setter
    def pinned(self, v):
        self.__pinned = v

    @property
    def coupled(self):
        return self.__coupled

    @coupled.setter
    def coupled(self, v):
        self.__coupled = v
        if v:
            self.__fixed = False

    @property
    def body(self):
        return self.__body

    @body.setter
    def body(self, v):
         self.__body= v
         if v:
             self.__coupled = False
             self.__fixed = True

    def __str__(self):
        """Get the string

        Returns
        -------
        str: The point string
        """
        if self.__body:
            value = "BODY" + str(self.__body.name)
            if self.__pinned:
                value = value + "PINNED"
            return value
        if self.__fixed:
            if self.__pinned:
                return "PINNED"
            return "FIXED"
        if not self.__coupled:
            return "FREE"
        if self.__pinned:
            return "CPLDPIN"
        return "COUPLED"


class Rod(Entity):
    """A rod
    """
    def __init__(self, material, point, p0, p1, n):
        """Constructor

        Parameters
        ----------
        material (RodMaterial): The rod material
        point (RodPoint): The rod point
        p0 (list): First point (3 components)
        p1 (list): Last point (3 components)
        n (int): Number of segments
        """

        Entity.__init__(
            self,
            "RODS",
            field_names=["ID", "RodType", "Type", "X0  Y0  Z0", "X1  Y1  Z1", "NumSegs", "Outputs"],
            field_units=["(#)", "(name)", "(-)", "(m) (m) (m)", "(m) (m) (m)", "(-)", "(-)"])
        self.__name = 0
        self.__mat = material
        self.__point = point
        self.__p0 = PropsList(p0, " ")
        self.__p1 = PropsList(p1, " ")
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
    def d(self, material):
        self.__mat = material
        self.__set_values()

    @property
    def point(self):
        return self.__point

    @point.setter
    def point(self, point):
        self.__point = point
        self.__set_values()

    @property
    def p0(self):
        return self.__p0

    @p0.setter
    def p0(self, p):
        self.__p0 = PropsList(p, " ")
        self.__set_values()

    @property
    def p1(self):
        return self.__p1

    @p1.setter
    def p1(self, p):
        self.__p1 = PropsList(p, " ")
        self.__set_values()

    @property
    def n(self):
        return self.__n

    @n.setter
    def n(self, n):
        self.__n = n
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__name,
                         self.__mat.name,
                         self.__point,
                         self.__p0,
                         self.__p1,
                         self.__n,
                         "-"])
