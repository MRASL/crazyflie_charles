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
import argparse
import rospy
import roslaunch

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


if __name__ == '__main__':
    # pylint: disable=invalid-name
    # snake_case naming is okay

    parser = argparse.ArgumentParser(description='Start crazyflie swarm')
    parser.add_argument('-n', type=int, help='Number of crazyflie in the swarm', default=1)
    parser.add_argument('--sim', '-s', action='store_true', help='Flag to launch in simulation')

    args = parser.parse_args()
    n_cf = args.n
    to_sim = args.sim

    # Launch server
    rospy.init_node('launchSwarm', anonymous=False)
    rospy.loginfo("Initializing server for %i crazyflies" % n_cf)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Create list /w name of the CFs
    cf_list = [('cf' + str(i + 1)) for i in range(n_cf)]

    # Launch server
    cli_server = ['crazyflie_charles', 'init_server.launch', 'cf_list:='+str(cf_list),
                  'to_sim:=%s' % to_sim]
    launch_file(cli_server)

    # Add n CFs
    for each_cf in cf_list:
        # TODO: Find uris of active CFs
        uri = 'radio://0/105/2M/0xE7E7E7E702'
        cli_add_cf = ['crazyflie_charles', 'add_cf.launch', 'cf_name:='+each_cf, 'uri:='+uri,
                      'frame:='+each_cf+'/'+each_cf, 'to_sim:=%s' % to_sim]
        launch_file(cli_add_cf)

    rospy.spin()
