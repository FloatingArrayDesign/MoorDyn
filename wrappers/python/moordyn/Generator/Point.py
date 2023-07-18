from .Entity import Entity, PropsList
from .Body import Body


class Point(Entity):
    """A point
    """
    def __init__(self, t, p, m, v, cda, ca):
        """Constructor

        Parameters
        ----------
        t (str): One of ("FIXED", "COUPLED", "FREE"), or a body
        p (list): Point (3 components)
        m (float): Node mass in the case of clump weights (kg)
        v (float): Node displacement in the case of floats (m^3)
        cda (float): Product of drag coefficient and projected area (assumed
                     constant in all directions) to calculate a drag force for
                     the node (m^2)
        ca (float): Added mass coefficient used along with V to calculate added
                    mass on node
        """
        Entity.__init__(
            self,
            "POINT PROPERTIES",
            field_names=["ID", "Type", "X   Y   Z", "Mass", "Volume", "CdA", "Ca"],
            field_units=["(#)", "(name)", "(m) (m) (m)", "(kg)", "(m^3)", "(m^2)", "(-)"])
        self.__name = 0
        self.__t = t
        self.__p = PropsList(p, " ")
        self.__m = m
        self.__v = v
        self.__cda = cda
        self.__ca = ca
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
    def mass(self):
        return self.__m

    @mass.setter
    def mass(self, v):
        self.__m = v
        self.__set_values()

    @property
    def vol(self):
        return self.__v

    @vol.setter
    def vol(self, v):
        self.__v = v
        self.__set_values()

    @property
    def cda(self):
        return self.__cda

    @cda.setter
    def cda(self, v):
        self.__cda = v
        self.__set_values()

    @property
    def ca(self):
        return self.__ca

    @ca.setter
    def ca(self, v):
        self.__ca = v
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__name,
                         self.__t,
                         self.__p,
                         self.__m,
                         self.__v,
                         self.__cda,
                         self.__ca])
