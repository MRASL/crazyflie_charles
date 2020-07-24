#!/usr/bin/env python

"""
Script to launch `.launch` files from Python.

Options:
    -n: To specify number of CFs in the swarm
    --sim: To launch in simulation

Example:
    ::

    $ rosrun crazyflie_charles launchSwarm.py

.._ Python api for launch files:
    http://wiki.ros.org/roslaunch/API%20Usage

"""

import sys
import rospy
import roslaunch

def launch_file(uuid, cli_args):
    """To execute launch files with arguments

    Args:
        cli_args (list - str): ["pkg_name", "file.launch", "args"]
    """
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    roslaunch_args = cli_args[2:]

    add_args(roslaunch_args)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

def add_args(arg_list):
    """Add arguments of the launch file

    Args:
        arg_list (list): List of args
    """
    for each_arg in arg_list:
        sys.argv.append(each_arg)

def get_args(group_name, args_dict):
    """Convert args read from yaml file to args readable by a launch file

    Args:
        group_name (str): Name of arguments groupe
        args_dict (dict): All args read from yaml file

    Returns:
        list of str: Converted args
    """
    args_list = []
    all_args = args_dict[group_name]
    for arg_name, arg_val in all_args.items():
        args_list.append(arg_name + ':=' + str(arg_val))

    return args_list

def launch_swarm(cf_list):
    """Launch each CF node.

    Args:
        cf_list (list of str): Name of all CF
    """

    print "Launching swarm"
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    first_uri = 0

    # Launch CFs
    base_address = 0xE7E7E7E700 + first_uri
    base_radio = 'radio://0/80/2M/'
    to_sim = rospy.get_param("~to_sim", "False")

    # Add n CFs
    for each_cf in cf_list:
        uri = base_radio + hex(base_address).upper()

        cli_add_cf = ['swarm_manager', 'add_cf.launch', 'cf_name:='+each_cf, 'uri:='+uri,
                      'frame:='+each_cf+'/'+each_cf, 'to_sim:=%s' % to_sim]

        launch_file(uuid, cli_add_cf)
        base_address = base_address + 1

def launch_joystick(joy_type):
    """Launch joystick controller

    Args:
        joy_type (str): Joystick type
    """
    print "Launching joystick controller"
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    cli_add_cf = ['swarm_manager', 'joy.launch', 'joy_type:=' + joy_type]

    launch_file(uuid, cli_add_cf)
