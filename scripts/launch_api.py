#!/usr/bin/env python

import roslaunch
import sys

def launch_file(uuid, cli_args):
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    roslaunch_args = cli_args[2:]

    add_args(roslaunch_args)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

def add_args(arg_list):
    for each_arg in arg_list:
        sys.argv.append(each_arg)