#!/usr/bin/env python

"""Test api
"""

import os
import time
import rospy

# pylint: disable=invalid-name
# pylint: disable=wrong-import-position
# pylint: disable=import-error

parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
api_path = os.path.join(parentdir, 'scripts')
os.sys.path.insert(0, api_path)

from swarm_api import SwarmAPI


def formation_test():
    """To test formation mode
    """
    print "Formation test"
    swarm.set_mode("formation")
    swarm.set_formation("line")
    swarm.take_off()
    time.sleep(7)
    print "Moving formation"
    swarm.go_to({'formation': [2, 2, 0.5, 0]})

def automatic_test():
    """To test automatic mode
    """
    swarm.set_mode("automatic")
    swarm.take_off()
    pose = swarm.get_positions()
    goals = {}
    goals["cf_0"] = pose["cf_1"]
    goals["cf_1"] = pose["cf_0"]
    swarm.go_to(goals)

if __name__ == "__main__":
    swarm = SwarmAPI()

    swarm.start_joystick()

    swarm.link_joy_button("S", swarm.take_off)
    swarm.link_joy_button("X", swarm.land)
    swarm.link_joy_button("O", swarm.emergency)
    swarm.link_joy_button("T", swarm.toggle_ctrl_mode)
    swarm.link_joy_button("R2", swarm.stop)


    swarm.link_joy_button("DL", swarm.prev_formation)
    swarm.link_joy_button("DR", swarm.next_formation)
    swarm.link_joy_button("DU", swarm.inc_scale)
    swarm.link_joy_button("DD", swarm.dec_scale)

    swarm.link_joy_button("L2", swarm.set_formation, "v")

    swarm.set_mode("automatic")
    # swarm.link_joy_button("L1", formation_test)
    swarm.link_joy_button("L1", automatic_test)

    rospy.spin()
