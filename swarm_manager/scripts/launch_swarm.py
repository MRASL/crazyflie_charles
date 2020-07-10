#!/usr/bin/env python

"""
Script to manage all the crazyflies

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
import os
import argparse
import rospy
import roslaunch
import yaml

def launch_file(cli_args):
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


if __name__ == '__main__':
    # pylint: disable=invalid-name
    # snake_case naming is ok

    parser = argparse.ArgumentParser(description='Start crazyflie swarm')
    parser.add_argument('-n', type=int, help='Number of crazyflie in the swarm', default=1)
    parser.add_argument('--sim', '-s', action='store_true', help='Flag to launch in simulation')

    args = parser.parse_args()
    n_cf = args.n
    to_sim = args.sim

    # Read arguments from yaml file
    parentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    file_path = os.path.join(parentdir, 'conf.yaml')

    with open(file_path) as f:
        yaml_conf = yaml.load(f, Loader=yaml.FullLoader)

    # Launch server
    rospy.init_node('launchSwarm', anonymous=False)
    rospy.loginfo("Initializing server for %i crazyflies" % n_cf)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Create list /w name of the CFs
    cf_list = [('cf' + str(i + 1)) for i in range(n_cf)]

    # Launch server
    cli_server = ['swarm_manager', 'init_server.launch', 'cf_list:='+str(cf_list),
                  'to_sim:=%s' % to_sim]

    server_args = get_args('crazyflie', yaml_conf)
    formation_args = get_args('formation', yaml_conf)
    cli_server = cli_server + server_args + formation_args

    launch_file(cli_server)

    # Launch CFs
    base_address = 0xE7E7E7E700
    base_radio = 'radio://0/80/2M/'
    cf_args = get_args('crazyflie', yaml_conf)

    starting_positions = yaml_conf['starting_positions']

    # Add n CFs
    for each_cf in cf_list:
        # TODO: Find uris of active CFs
        uri = base_radio + hex(base_address).upper()
        try:
            starting_pos = starting_positions[each_cf]
            starting_pos = 'starting_position:=%s' % starting_pos
        except KeyError:
            starting_pos = None

        cli_add_cf = ['swarm_manager', 'add_cf.launch', 'cf_name:='+each_cf, 'uri:='+uri,
                      'frame:='+each_cf+'/'+each_cf, 'to_sim:=%s' % to_sim]
        cli_add_cf = cli_add_cf + cf_args

        if starting_pos is not None:
            cli_add_cf.append(starting_pos)

        launch_file(cli_add_cf)
        base_address = base_address + 1

    rospy.spin()
