from .Entity import Entity, PropsList


class BodyPoint():
    """The body point
    """
    def __init__(self):
        self.__fixed = True
        self.__coupled = False

    @property
    def fixed(self):
        return self.__fixed

    @fixed.setter
    def fixed(self, v):
        self.__fixed = v
        if v:
            self.__coupled = False

    @property
    def coupled(self):
        return self.__coupled

    @coupled.setter
    def coupled(self, v):
        self.__coupled = v
        if v:
            self.__fixed = False

    def __str__(self):
        """Get the string

        Returns
        -------
        str: The point string
        """
        if self.__fixed:
            return "FIXED"
        if not self.__coupled:
            return "FREE"
        return "COUPLED"


class Body(Entity):
    """A body
    """
    def __init__(self, t, p, r, m, cog, inertia, cda, v):
        """Constructor

        Parameters
        ----------
        t (BodyPoint): The type of body point
        p (list): Point (3 components)
        r (list): Rotation (3 components, radians)
        m (float): Node mass in the case of clump weights (kg)
        cog (float/list): The center of gravity (3 components). If just a single
                          number is provided, then (0, 0, cog) is considered
        v (float): Node displacement in the case of floats (m^3)
        inertia (float/list): The inertia (3 components). If just a single
                              number is provided, then (inertia, inertia, inertia)
                              is considered
        cda (float/list): Drag coefficients (3 components). If just a single
                          number is provided, then (cda, cda, cda) is considered
        ca (float/list): Added mass coefficients (3 components). If just a single
                         number is provided, then (ca, ca, ca) is considered
        """
        Entity.__init__(
            self,
            "BODIES",
            field_names=["ID", "Type", "X   Y   Z  ", "Roll  Pitch Yaw", "Mass", "CoG", "Volume", "Inertia", "CdA", "Ca"],
            field_units=["(#)", "(name)", "(m) (m) (m)", "(rad) (rad) (rad)", "(kg)", "(m)", "(m^3)", "(kg*m^2)", "(m^2)", "(-)"])
        self.__name = 0
        self.__t = t
        self.__p = PropsList(p, " ")
        self.__r = PropsList(r, " ")
        self.__m = m
        self.__cog = PropsList(cog, "|")
        self.__v = v
        self.__I = PropsList(inertia, "|")
        self.__cda = PropsList(cda, "|")
        self.__ca = PropsList(ca, "|")
        self.__set_values()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, n):
        self.__name = n
        self.__set_values()

    @property
    def type(self):
        return self.__t

    @type.setter
    def type(self, t):
        self.__t = t
        self.__set_values()

    @property
    def p(self):
        return self.__p

    @p.setter
    def p(self, p):
        self.__p = PropsList(p, " ")
        self.__set_values()

    @property
    def r(self):
        return self.__r

    @r.setter
    def r(self, r):
        self.__r = PropsList(r, " ")
        self.__set_values()

    @property
    def mass(self):
        return self.__m

    @mass.setter
    def mass(self, v):
        self.__m = v
        self.__set_values()

    @property
    def cog(self):
        return self.__cog

    @cog.setter
    def cog(self, v):
        self.__cog = PropsList(v, "|")
        self.__set_values()

    @property
    def vol(self):
        return self.__v

    @vol.setter
    def vol(self, v):
        self.__v = v
        self.__set_values()

    @property
    def I(self):
        return self.__I

    @I.setter
    def I(self, v):
        self.__I = PropsList(v, "|")
        self.__set_values()

    @property
    def cda(self):
        return self.__cda

    @cda.setter
    def cda(self, v):
        self.__cda = PropsList(v, "|")
        self.__set_values()

    @property
    def ca(self):
        return self.__ca

    @ca.setter
    def ca(self, v):
        self.__ca = PropsList(v, "|")
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__name,
                         self.__t,
                         self.__p,
                         self.__r,
                         self.__m,
                         self.__cog,
                         self.__v,
                         self.__I,
                         self.__cda,
                         self.__ca])
