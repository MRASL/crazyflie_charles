"""Demo script for MRASL lab.

Moves 5 crazyflie in a different formations

Press 'O' in case of emergency.
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

    print "MRASL demo"
    swarm.set_mode("formation")
    swarm.set_formation("line")

    rospy.sleep(2)
    print "Taking off"
    print "Formation: Line"
    swarm.take_off()
    rospy.sleep(10)

    print "Formation: V"
    swarm.set_formation("v")
    rospy.sleep(10)

    print "Formation: Circle"
    swarm.set_formation("circle")
    rospy.sleep(10)

    print "Turning"
    rospy.sleep(10)


    print "Formation: Pyramid"
    swarm.set_formation("pyramid")
    rospy.sleep(10)

    print "Landing to start positions"
    swarm.land()

    # print "Moving formation to (2, 2, 0.5)"
    # swarm.go_to({'formation': [2, 2, 0.5, 0]})
