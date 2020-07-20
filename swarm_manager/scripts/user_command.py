"""Module for easier interpretation of user commands

.. warning::
    Args need to be added in order.
    No '' around str vals.

Kwargs are optional

"""

from collections import OrderedDict

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

        if func_name == "help":
            self.print_help()

        else:
            if len(cmd_input) > 1:
                func_args = cmd_input[1::]

            if func_name in self.cmd_list.keys():
                func_cmd = self.cmd_list[func_name]
            else:
                raise InputError("Function %s is invalid" % func_name)

            func = func_cmd.func
            args, kwargs = split_args(func_args)
            args, kwargs = func_cmd.check_args(args, kwargs)

        return func, args, kwargs

    def print_help(self):
        """Print all possible commands and their arguments
        """
        print "Possible commands:"
        print "------------------"
        for cmd_name, cmd in self.cmd_list.items():
            print "%s: %s" % (cmd_name, cmd.description)
            print "\tArgs:"
            for arg_name, arg_info in cmd.args.items():
                print "\t\t%s: %s (%s)" % (arg_name, arg_info["description"],
                                           arg_info["type"].__name__)
            print "\tKwargs:"
            for kwarg_name, kwarg_info in cmd.kwargs.items():
                print "\t\t%s: %s (%s)" % (kwarg_name, kwarg_info["description"],
                                           kwarg_info["type"].__name__)
        print "------------------"


class Command(object):
    """
    Class that holds information for a command
    """
    def __init__(self, func, description):
        self.func = func
        self.description = description
        self.args = OrderedDict()
        self.kwargs = {}

    def add_arg(self, arg_name, description=None, arg_type=None, optional=False):
        """Add an arg to command

        Args:
            arg_name (str): arg name
            description (str, optional): Description of arg. Defaults to None.
            arg_type (str, optional): Type of arg. Defaults to None.
            optional (bool, optional): If arg is optional in function call. Defaults to False.
        """
        if optional:
            self.kwargs[arg_name] = {'description': description, 'type': arg_type}
        else:
            self.args[arg_name] = {'description': description, 'type': arg_type}

    def check_args(self, args, kwargs):
        """Verify if all required args were passed.

        Also verify type of args

        Args:
            args (list): Args list
            kwargs (dict): Kwargs
        """
        # Check args and kwargs are valid
        args_ok = len(args) == len(self.args.keys())
        if not args_ok:
            raise InputError("Invalid number of args")

        for key in kwargs.keys():
            if key not in self.kwargs.keys():
                raise InputError("%s is an invalid keyword" % key)

        # Check type
        args_corr = []
        kwargs_corr = {}

        for arg, (_, arg_info) in zip(args, self.args.items()):
            args_corr.append(arg_info['type'](arg))

        for key, val in kwargs.items():
            arg_type = self.kwargs[key]['type']
            kwargs_corr[key] = arg_type(val)

        return args_corr, kwargs_corr


class InputError(Exception):
    """Exception raised for errors in the input
    """
    pass


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
    args = []
    kwargs = {}

    if all_args is not None:
        args = [a for a in all_args if ':=' not in a]
        kwargs_list = [a for a in all_args if ':=' in a]

        for each_arg in kwargs_list:
            vals = each_arg.split(':=')
            kwargs[vals[0]] = vals[1]

    return args, kwargs
