#!/usr/bin/env python

"""Demo script to test hovering of a CF.

Press 'O' in case of emergency.

Take off with Square, Land with X.
"""

import os
import rospy

# pylint: disable=invalid-name
# pylint: disable=wrong-import-position
# pylint: disable=import-error

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
api_path = os.path.join(parentdir, 'scripts')
os.sys.path.insert(0, api_path)

from swarm_api import SwarmAPI

def init_joystick():
    """Start joystick and link buttons
    """
    swarm.start_joystick()

    swarm.link_joy_button("S", swarm.take_off)
    swarm.link_joy_button("X", swarm.land)
    swarm.link_joy_button("O", swarm.emergency)

if __name__ == "__main__":
    swarm = SwarmAPI()

    init_joystick()

    print "Hover demo"
    print "Possible input:"
    print "\tO: Emergency"
    print "\tSquare: Take Off"
    print "\tX: Land"

    rospy.spin()
