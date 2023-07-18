class Entity:
    """Base class for all the other entities
    """
    def __init__(self,
                 section_name,
                 extra_header="",
                 field_names=[],
                 field_units=[]):
        """Constructor

        Parameters
        ----------
        section_name (str): The section name

        Keyword arguments
        -----------------
        extra_header (str): String to be added right after the header line.
                            Usually just the Intro section might accept that.
        field_names (list): The list of field names
        field_units (list): The list of field units
        """
        self.__name = section_name
        self.__extra_header = extra_header
        self.__field_names = field_names
        self.__field_units = field_units
        self.__values = None
        self.__field_widths = self.get_field_widths()

    def set_values(self, values):
        """Set the field values

        Parameters
        ----------
        values (list): The list of values
        """
        self.__values = values
        self.__field_widths = self.get_field_widths()

    def set_field_widths(self, widths):
        """Set the widths

        Parameters
        ----------
        widths (list): The list of strig width for each field
        """
        self.__field_widths = widths

    def get_field_widths(self):
        """Get the minimum width required to print the fields

        Returns
        -------
        list: The list of widths, which might not match the one set with
              set_field_widths() if more space is required for a field
        """
        widths = [len(field) + 1 for field in self.__field_names]
        for i, unit in enumerate(self.__field_units):
            widths[i] = max(widths[i], len(unit) + 1)
        if self.__values:
            for i in range(len(self.__values) - len(widths)):
                # For the cases without field names
                widths.append(0)
            for i, value in enumerate(self.__values):
                widths[i] = max(widths[i], len(str(value)) + 1)
        try:
            for i, w in enumerate(self.__field_widths):
                widths[i] = max(widths[i], w)
        except AttributeError:
            pass
        return widths

    def get_header(self):
        """Get the header string

        Returns
        -------
        str: The header string. This should be asked just for the first entity
        """
        hdr = (" " + self.__name + " ").center(80, "-") + "\n"
        if not hdr.startswith("---"):
            hdr = "---" + hdr
        hdr = hdr + self.__extra_header
        if not hdr.endswith("\n"):
            hdr = hdr + "\n"

        for i, name in enumerate(self.__field_names):
            hdr = hdr + name.ljust(self.__field_widths[i], " ")
        if not hdr.endswith("\n"):
            hdr = hdr + "\n"

        for i, unit in enumerate(self.__field_units):
            hdr = hdr + unit.ljust(self.__field_widths[i], " ")
        if not hdr.endswith("\n"):
            hdr = hdr + "\n"
        return hdr

    def get_values(self):
        """Get the string of values

        Returns
        -------
        str: The values string
        """
        values = ""
        for i, value in enumerate(self.__values):
            values = values + str(value).ljust(self.__field_widths[i], " ")
        if not values.endswith("\n"):
            values = values + "\n"
        return values


def to_list(v):
    try:
        return list(v)[:]
    except TypeError:
        return [v]


class PropsList:
    """The line point
    """
    def __init__(self, l, sep):
        """Constructor

        Parameters
        ----------
        l (list): The list
        sep (str): Fields separator
        """
        self.__values = to_list(l)
        self.__sep = sep

    @property
    def values(self):
        return self.__values

    @values.setter
    def values(self, v):
        self.__values = to_list(v)

    @property
    def separator(self):
        return self.__end_point

    @separator.setter
    def separator(self, v):
        self.__separator = v

    def __str__(self):
        """Get the string

        Returns
        -------
        str: The string
        """
        out = ""
        for value in self.__values[:-1]:
            out = out + str(value) + self.__sep
        out = out + str(self.__values[-1])
        return out
