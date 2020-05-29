#!/usr/bin/env python

"""
To manage the flight of the swarm
"""

import rospy
import tf
from crazyflie import Crazyflie
from crazyflie_sim import CrazyflieSim

from geometry_msgs.msg import Pose
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest

# TODO Parameters update
class Swarm:
    def __init__(self, cf_list, to_sim):
        # Dict for all the cf and their functions/parameters
        #   - Keys: cf_names
        #       - Dict:
        #           - cf: Crazyflie instance
        #           - emergency service
        #           - goal_publisher
        #           - get_pose
        #           - initial_pose

        self.crazyflies = {}
        self.crazyflies_sim = {}

        self._emergency_srv_list = []
        self._goal_publisher_list = []
        
        self._get_pose_srv_list = []
        self._initial_pose_list = []
        
        # Initialize each Crazyflie
        for each_cf in cf_list:
            self.crazyflies[each_cf] = {"emergency_srv": None,
                                        "goal_pub": None,
                                        "get_pose_srv": None,
                                        "initial_pose": None,
                                        "take_off": None,
                                        "hover": None,
                                        "land": None,
                                        "stop": None,
                                        "toggle_teleop": None}

            # if to_sim:
            #     self.crazyflies_sim[each_cf] = {"cf": CrazyflieSim(each_cf)}

            if not to_sim:
                # Subscribe to emergency service
                rospy.loginfo("Swarm: waiting for emergency service of " + each_cf)
                rospy.wait_for_service('/' + each_cf + '/emergency')
                rospy.loginfo("Swarm: found emergency service of " + each_cf)
                self.crazyflies[each_cf]["emergency_srv"] = rospy.ServiceProxy('/' + each_cf + '/emergency', srv.Empty)

            # Subscribe to services
            # TODO: Add other services
            rospy.loginfo("Swarm: waiting for pose service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/get_pose')
            rospy.loginfo("Swarm: found pose service of " + each_cf)
            self.crazyflies[each_cf]["get_pose_srv"] = rospy.ServiceProxy('/' + each_cf + '/get_pose', PoseRequest)

            # Publish goal
            self.crazyflies[each_cf]["goal_pub"] = rospy.Publisher('/' + each_cf + '/goal', Pose, queue_size=1)

        # Launch services
        rospy.Service('/update_params', srv.Empty, self.update_params)      # TODO: #14 Update all parameters
        rospy.Service('/emergency', srv.Empty, self.emergency)              # Emergency
        rospy.Service('/stop', srv.Empty, self.stop)                        # Stop all CFs
        rospy.Service('/takeoff', srv.Empty, self.takeOff)                  # Take off all CFs
        rospy.Service('/land', srv.Empty, self.land)                        # Land all CFs
        rospy.Service('/toggleTeleop', srv.Empty, self.toggleTeleop)        # Toggle between manual and auto mode
        rospy.Service('/getSwarmPose', PoseRequest, self.get_swarm_pose)    # Find position of the swarm

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
            cf["cf"].take_off()

        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")
        for _, cf in self.crazyflies.items(): 
            cf["cf"].land()
        return srv.EmptyResponse()
    
    def broadcast_goal(self, goal):
        # TODO: Different goal for each CF depending on formation (#15)
        for _, cf in self.crazyflies.items(): 
            cf["goal_pub"].publish(goal)

    def get_swarm_pose(self, req):
        swarmPose = Pose()
        for _, cf in self.crazyflies.items(): 
            cf["initial_pose"] = cf["get_pose_srv"]()
            swarmPose = cf["initial_pose"]
            
        # TODO: #15 Initial swarm position depending on formation
        # For one CF swarm pos = CF pos
        return swarmPose

    # Run in automatic
    def run_auto(self):
        for _, cf in self.crazyflies.items():  # _: Key (cf_id) cf: items dict
            cf["cf"].run_auto()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')

    # Get params
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    to_sim = rospy.get_param("~to_sim", "False")
    
    # Initialize swarm
    swarm = Swarm(cf_list, to_sim)

    rospy.spin()