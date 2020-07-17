"""Module for easier interpretation of user commands
"""

class UserInterface(object):
    """ To determine function entered by user
    """
    def __init__(self):
        self.cmd_list = {}

    def add_cmd(self, cmd_name, func, description=None):
        """Add a command

        Args:
            cmd_name (str): What the user has to type
            func (address): Address of function to call
            description (str, optional): Description of command. Defaults to None.
        """
        self.cmd_list[cmd_name] = Command(func, description)

    def add_arg(self, cmd_name, arg_name, description=None, arg_type=None, optional=False):
        """Add argument to a command

        Args:
            cmd_name (str): Name of command
            arg_name (str): Name of arg
            description (str, optional): Arg description. Defaults to None.
            arg_type (str, optional): Arg type. Defaults to None.
            optional (bool, optional): If arg is optional. Defaults to False.
        """
        self.cmd_list[cmd_name].add_arg(arg_name, description, arg_type, optional)

    def get_command(self, cmd_input):
        """Find command function, args and kwargs

        Args:
            cmd_input (str): User input

        Raises:
            LookupError: When user enters an invalid cmd

        Returns:
            list: func, args, kwargs
        """

        func = None
        args = None
        kwargs = None
        func_args = None

        cmd_input = cmd_input.split(' ')

        func_name = cmd_input[0]
        if len(cmd_input) > 1:
            func_args = cmd_input[1::]

        if func_name in self.cmd_list.keys():
            func_cmd = self.cmd_list[func_name]
        else:
            raise LookupError("Function %s is invalid" % func_name)

        func = func_cmd.func
        args, kwargs = split_args(func_args)

        # TODO Check if there are missing args

        # TODO Check types

        return func, args, kwargs

    def print_help(self):
        """Print all possible commands and their arguments
        """
        # TODO Print help
        pass

class Command(object):
    """
    Class that holds information for a command
    """
    def __init__(self, func, description):
        self.func = func
        self.description = description
        self.args = {}

    def add_arg(self, arg_name, description=None, arg_type=None, optional=False):
        """Add an arg to command

        Args:
            arg_name (str): arg name
            description (str, optional): Description of arg. Defaults to None.
            arg_type (str, optional): Type of arg. Defaults to None.
            optional (bool, optional): If arg is optional in function call. Defaults to False.
        """
        self.args[arg_name] = {'description': description, 'type': arg_type, 'optional': optional}


def split_args(all_args):
    """Split incoming args

    All args need to be separted by a space. Kwargs need to have ':=' between key and val

    i.e:
        ``arg1 arg2 kwarg1:=val kwarg2:='ex'``

    Args:
        all_args (list of str): List of input args

    Returns:
        list: args, kwargs
    """
    args = [a for a in all_args if ':=' not in a]
    kwargs_list = [a for a in all_args if ':=' in a]
    kwargs = {}

    for each_arg in kwargs_list:
        vals = each_arg.split(':=')
        kwargs[vals[0]] = vals[1]

    return args, kwargs
