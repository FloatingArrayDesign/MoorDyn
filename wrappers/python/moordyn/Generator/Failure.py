from .Entity import Entity, PropsList
from .Line import LineConnection


class Failure(Entity):
    """A failure criteria
    """
    def __init__(self, con, lines, time, ten):
        """Constructor

        Parameters
        ----------
        con (LineConnection): The Line connection
        lines (list): The lines affected
        time (float): The failure tme delay
        ten (float): The failure tension
        """

        Entity.__init__(
            self,
            "FAILURE",
            field_names=["Attach", "Lines", "FailTime", "FailTen"],
            field_units=["(#)", "(#)", "(s)", "(N)"])
        self.__con = con
        self.__lines = lines
        self.__time = time
        self.__ten = ten
        self.__set_values()

    @property
    def con(self):
        return self.__con

    @con.setter
    def con(self, connection):
        self.__con = connection
        self.__set_values()

    @property
    def lines(self):
        return self.__lines

    @lines.setter
    def lines(self, lines):
        self.__lines = lines
        self.__set_values()

    @property
    def time(self):
        return self.__time

    @time.setter
    def time(self, v):
        self.__time = v
        self.__set_values()

    @property
    def ten(self):
        return self.__ten

    @ten.setter
    def ten(self,ten):
        self.__ten = ten
        self.__set_values()

    def __set_values(self):
        l = PropsList([line.name for line in self.__lines], ",")
        self.set_values([self.__con,
                         PropsList([line.name for line in self.__lines], ","),
                         self.__time,
                         self.__ten])
