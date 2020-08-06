"""Demo script to move swarm in formation.

Press 'O' in case of emergency.

Move formation with joystick sticks
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

    swarm.link_joy_button("S", swarm.take_off)
    swarm.link_joy_button("X", swarm.land)
    swarm.link_joy_button("O", swarm.emergency)
    swarm.link_joy_button("T", swarm.toggle_ctrl_mode)

    swarm.link_joy_button("DL", swarm.prev_formation)
    swarm.link_joy_button("DR", swarm.next_formation)
    swarm.link_joy_button("DU", swarm.inc_scale)
    swarm.link_joy_button("DD", swarm.dec_scale)

    swarm.link_joy_button("L2", swarm.set_formation, "v")

if __name__ == "__main__":
    swarm = SwarmAPI()


    init_joystick()

    print "Formation demo"
    swarm.set_mode("formation")
    swarm.set_formation("line")

    swarm.take_off()
    rospy.sleep(10)

    print "Next formation"
    swarm.set_formation("v")
    rospy.sleep(10)

    print "Next formation"
    swarm.set_formation("pyramid")
    rospy.sleep(10)

    print "Next formation"
    swarm.set_formation("circle")
    rospy.sleep(10)

    print "Next formation"
    swarm.set_formation("square")
    rospy.sleep(10)

    # print "Moving formation to (2, 2, 0.5)"
    # swarm.go_to({'formation': [2, 2, 0.5, 0]})

    rospy.spin()
