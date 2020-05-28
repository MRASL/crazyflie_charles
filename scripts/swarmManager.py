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

class Swarm:
    def __init__(self, cf_list):
        # Dict for all the cf and their functions/parameters
        self.crazyflies = {}

        self._emergency_srv_list = []
        self._goal_publisher_list = []
        
        self._get_pose_srv_list = []
        self._initial_pose_list = []
        
        for each_cf in cf_list:
            self.crazyflies[each_cf] = 

            # Subscribe to emergency service
            rospy.loginfo("waiting for emergency service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/emergency')
            rospy.loginfo("found emergency service of " + each_cf)
            self._emergency_srv_list.append(rospy.ServiceProxy('/' + each_cf + '/emergency', srv.Empty))

            # Subscribe to pose service
            rospy.loginfo("waiting for pose service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/get_pose')
            rospy.loginfo("found pose service of " + each_cf)
            self._get_pose_srv_list.append(rospy.ServiceProxy('/' + each_cf + '/get_pose', PoseRequest))

            # Publish goal
            self._goal_publisher_list.append(rospy.Publisher('/' + each_cf + '/goal', Pose, queue_size=1))

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
        for each_emergency in self._emergency_srv_list:
            each_emergency()

        return srv.EmptyResponse()

    def stop(self, req):
        rospy.loginfo("Swarm: stop")
        for cf in self.crazyflies: cf.stop()
        return srv.EmptyResponse()
    
    def takeOff(self, req):
        rospy.loginfo("Swarm: take off")
        # for cf in self.crazyflies: 
        #     cf.take_off()

        for get_pose in self._get_pose_srv_list:
            self._initial_pose_list.a

        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")
        for cf in self.crazyflies: cf.land()
        return srv.EmptyResponse()
    
    def broadcast_goal(self, goal):
        for each_pub in self._goal_publisher_list:
            each_pub.publish(goal)

    def run_auto(self):
        for cf in self.crazyflies: cf.run_auto()


if __name__ == '__main__':
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    swarm = Swarm(cf_list)

    while not rospy.is_shutdown():
        if not swarm.in_teleop():
            swarm.run_auto()