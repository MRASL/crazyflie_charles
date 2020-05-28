#!/usr/bin/env python

"""
To manage the flight of the swarm
"""

import rospy
import tf
from crazyflie import Crazyflie

from geometry_msgs.msg import Pose
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest

# TODO Parameters update
class Swarm:
    def __init__(self, cf_list):
        # Dict for all the cf and their functions/parameters
        #   - Keys: cf_names
        #       - Dict:
        #           - cf: Crazyflie instance
        #           - emergency service
        #           - goal_publisher
        #           - get_pose
        #           - initial_pose

        self.crazyflies = {}

        self._emergency_srv_list = []
        self._goal_publisher_list = []
        
        self._get_pose_srv_list = []
        self._initial_pose_list = []
        
        for each_cf in cf_list:
            self.crazyflies[each_cf] = {"cf": Crazyflie(each_cf),
                                        "emergency_srv": None,
                                        "goal_pub": None,
                                        "get_pose_srv": None,
                                        "initial_pose": None}

            # Subscribe to emergency service
            rospy.loginfo("Swarm: waiting for emergency service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/emergency')
            rospy.loginfo("Swarm: found emergency service of " + each_cf)

            self.crazyflies[each_cf]["emergency_srv"] = rospy.ServiceProxy('/' + each_cf + '/emergency', srv.Empty)

            # Subscribe to pose service
            rospy.loginfo("Swarm: waiting for pose service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/get_pose')
            rospy.loginfo("Swarm: found pose service of " + each_cf)
            self.crazyflies[each_cf]["get_pose_srv"] = rospy.ServiceProxy('/' + each_cf + '/get_pose', PoseRequest)

            # Publish goal
            self.crazyflies[each_cf]["goal_pub"] = rospy.Publisher('/' + each_cf + '/goal', Pose, queue_size=1)

        # Launch services
        rospy.Service('/update_params', srv.Empty, self.update_params)
        rospy.Service('/emergency', srv.Empty, self.emergency)
        rospy.Service('/stop', srv.Empty, self.stop)
        rospy.Service('/takeoff', srv.Empty, self.takeOff)   
        rospy.Service('/land', srv.Empty, self.land)      
        rospy.Service('/toggleTeleop', srv.Empty, self.toggleTeleop)   

        # Subscribe
        rospy.Subscriber("goal_swarm", Pose, self.broadcast_goal)

        self._to_teleop = False   

    # Setter & getters
    def in_teleop(self):
        return self._to_teleop
        
    # Services methods
    def toggleTeleop(self, req):
        self._to_teleop = not self._to_teleop
        return srv.EmptyResponse()

    def update_params(self, req):
        rospy.loginfo("Swarm: Update params")
        return srv.EmptyResponse()

    def emergency(self, req):
        rospy.logerr("Swarm: EMERGENCY")
        for _, cf in self.crazyflies.items(): 
            cf["emergency_srv"]()

        return srv.EmptyResponse()

    def stop(self, req):
        rospy.loginfo("Swarm: stop")
        for _, cf in self.crazyflies.items(): 
            cf["cf"].stop()

        return srv.EmptyResponse()
    
    def takeOff(self, req):
        rospy.loginfo("Swarm: take off")

        for _, cf in self.crazyflies.items(): 
            print(cf["get_pose_srv"]())
            cf["cf"].take_off()

        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")
        for _, cf in self.crazyflies.items(): 
            cf["cf"].land()
        return srv.EmptyResponse()
    
    def broadcast_goal(self, goal):
        for _, cf in self.crazyflies.items(): 
            cf["goal_pub"].publish(goal)

    def run_auto(self):
        for _, cf in self.crazyflies.items():  # _: Key (cf_id) cf: items dict
            cf["cf"].run_auto()

if __name__ == '__main__':
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    swarm = Swarm(cf_list)

    while not rospy.is_shutdown():
        if not swarm.in_teleop():
            swarm.run_auto()