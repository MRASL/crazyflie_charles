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
