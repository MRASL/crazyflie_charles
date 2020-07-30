#!/usr/bin/env python

"""Demo script to take off 2 CFs before switching their position.

Press 'O' in case of emergency
"""
# pylint: disable=invalid-name

import rospy

# pylint: disable=no-name-in-module
# pylint: disable=import-error
from swarm_api.api import SwarmAPI
# pylint: enable=no-name-in-module
# pylint: enable=import-error


def init_joystick():
    """Start joystick and link buttons
    """
    swarm.start_joystick("ds4")

    swarm.link_joy_button("O", swarm.emergency)

if __name__ == "__main__":
    swarm = SwarmAPI()
    init_joystick()

    swarm.set_mode("automatic")

    print "Taking off"
    swarm.take_off()

    # Switch positions
    pose = swarm.get_positions()
    goals = {}
    goals["cf_0"] = pose["cf_1"]
    goals["cf_1"] = pose["cf_0"]
    swarm.go_to(goals)

    rospy.sleep(10)

    print "Landing"
    swarm.land()
