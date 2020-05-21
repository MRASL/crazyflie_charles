#!/usr/bin/env python

"""
Script to manage all the crazyflies

Python api for launch files: http://wiki.ros.org/roslaunch/API%20Usage
"""

import rospy
import roslaunch
import sys

def launch_file(cli):
    roslaunch_server = roslaunch.rlutil.resolve_launch_arguments(cli)



def add_args(arg_list):
    for each_arg in arg_list:
        sys.argv.append(each_arg)


if __name__ == '__main__':
    rospy.init_node('init_server', anonymous=True)
    
    rospy.loginfo("Initialising server")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)


    cli_server = ['crazyflie_charles', 'init_server.launch']
    cli_add_cf = ['crazyflie_charles', 'add_cf.launch', 'uri:=radio://0/105/2M/0xE7E7E7E701', 'frame:=crazyflie1/crazyflie1']
    
    roslaunch_server = roslaunch.rlutil.resolve_launch_arguments(cli_server)

    roslaunch_add_cf = roslaunch.rlutil.resolve_launch_arguments(cli_add_cf)
    roslaunch_add_cf_args = cli_add_cf[2:]


    launch_server = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_server)
    launch_server.start()

    add_args(roslaunch_add_cf_args)
    launch_add_cf = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_add_cf)
    launch_add_cf.start()

    rospy.spin()

