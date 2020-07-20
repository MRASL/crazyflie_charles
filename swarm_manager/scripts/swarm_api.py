"""Python API to control the swarm
"""

import rospy


class SwarmAPI(object):
    """API class to control the swarm
    """
    def __init__(self):
        rospy.init_node("CrazyflieAPI", anonymous=False)
