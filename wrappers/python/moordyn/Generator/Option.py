from .Entity import Entity


class Option(Entity):
    """An option
    """
    def __init__(self, name, value):
        """Constructor

        Parameters
        ----------
        name (str): The option name
        value (number): The option value
        """
        Entity.__init__(
            self,
            "OPTIONS")
        self.__name = name
        self.__value = value
        self.__set_values()

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, n):
        self.__name = n
        self.__set_values()

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, value):
        self.__value = value
        self.__set_values()

    def __set_values(self):
        self.set_values([self.__value, self.__name])
