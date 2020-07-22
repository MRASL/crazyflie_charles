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

if __name__ == "__main__":
    swarm = SwarmAPI()

    swarm.start_joystick()

    swarm.link_joy_button("S", swarm.take_off)
    swarm.link_joy_button("X", swarm.land)
    swarm.link_joy_button("O", swarm.emergency)
    # swarm.link_joy_button("T", swarm.toggle_ctrl_mode)
    swarm.link_joy_button("R2", swarm.stop)

    swarm.link_joy_button("DL", swarm.prev_formation)
    swarm.link_joy_button("DR", swarm.next_formation)
    swarm.link_joy_button("DU", swarm.inc_scale)
    swarm.link_joy_button("DD", swarm.dec_scale)



    # print "Take off"
    # swarm.take_off()
    # time.sleep(10)

    # print "Circle"
    # swarm.next_formation()
    # time.sleep(5)

    # swarm.inc_scale()
    # time.sleep(5)

    # print "Land"
    # swarm.land()

    rospy.spin()
