#!/usr/bin/env python

"""
Script to manage all the crazyflies

Python api for launch files: http://wiki.ros.org/roslaunch/API%20Usage
"""

import rospy
import roslaunch
import sys

def launch_file(cli_args):
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    roslaunch_args = cli_args[2:]

    add_args(roslaunch_args)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
    launch.start()

def add_args(arg_list):
    for each_arg in arg_list:
        sys.argv.append(each_arg)

if __name__ == '__main__':
    rospy.init_node('init_server', anonymous=True)
    
    rospy.loginfo("Initialising server")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)


    cli_server = ['crazyflie_charles', 'init_server.launch']

    #TODO: Add input for # of cf
    #TODO: Add input for control methode (man or auto)
    cf_name = 'crazyflie1'
    cli_add_cf = ['crazyflie_charles', 'add_cf.launch', 'to_teleop:=true', 'cf_name:='+cf_name, 'uri:=radio://0/105/2M/0xE7E7E7E701', 'frame:='+cf_name+'/'+cf_name]
    
    launch_file(cli_server)
    launch_file(cli_add_cf)

    rospy.spin()

