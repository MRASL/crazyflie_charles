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
    swarm.start_joystick("ds4", joy_dev='js1')
    swarm.set_joy_control(False)  # To control formation position with joystick

    swarm.link_joy_button("O", swarm.emergency)

if __name__ == "__main__":
    swarm = SwarmAPI()

    init_joystick()

    print "MRASL demo"
    swarm.set_mode("formation")
    swarm.set_formation("v")

    rospy.sleep(3)
    print "Taking off"
    print "Formation: v"
    swarm.take_off()
    rospy.sleep(10)

    print "Formation: Circle"
    swarm.set_formation("circle")
    rospy.sleep(10)

    print "Turning"
    swarm.rotate_formation(180, 5)
    rospy.sleep(2)

    print "Formation: Pyramid"
    swarm.set_formation("pyramid")
    rospy.sleep(10)

    print "Landing to start positions"
    swarm.land()
