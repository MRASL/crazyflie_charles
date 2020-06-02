#!/usr/bin/env python

"""
To manage the flight of the swarm.

Args:
    cf_list (list of str): Name of all CFs in the swarm
    to_sim (bool): To simulate or run on physical robots

TODO:
    * Update parameters
"""

import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import numpy as np

from crazyflie import Crazyflie
from crazyflie_sim import CrazyflieSim
from swarmFormation import SquareFormation, SingleFormation

from geometry_msgs.msg import Pose, Twist, Quaternion
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest, PoseSet

TAKE_OFF_DZ = 0.5 #: (float) Take off height in meters
GND_HEIGHT = 0.2 #: (float) Height of the ground

class Swarm:
    """Controls the swarm """

    def __init__(self, cf_list, to_sim):
        """
        Args:
            cf_list (list of str): Name of all CFs in the swarm
            to_sim (bool): To simulate or not
        """
        self.crazyflies = {} #: dict: Keys are name of the CF
        self.crazyflies_sim = {}
        self._to_sim = to_sim
        self._to_teleop = False   
        self.rate = rospy.Rate(100)
        self.cf_list = cf_list
        self.swarm_goal = Pose()
        self.swarm_pose = Pose() #: Position of the swarm

        self.swarm_goal.orientation.w = 1
        self.swarm_pose.orientation.w = 1
        
        # Change class depending on formation
        self.formation = SquareFormation(self.cf_list, offset=[0, 0, 0.2])
        # self.formation = SingleFormation(self.cf_list, offset=[0.2, 0.2, 0.2])
        
        
        self.formation.compute_initial_pose()

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self._init_cf(each_cf)

        # Launch services
        rospy.Service('/update_params', srv.Empty, self.update_params)      # TODO: #14 Update all parameters
        rospy.Service('/emergency', srv.Empty, self.emergency)              # Emergency
        rospy.Service('/stop', srv.Empty, self.stop)                        # Stop all CFs
        rospy.Service('/takeoff', srv.Empty, self.takeOff)                  # Take off all CFs
        rospy.Service('/land', srv.Empty, self.land)                        # Land all CFs
        rospy.Service('/toggleTeleop', srv.Empty, self.toggleTeleop)        # Toggle between manual and auto mode

        # Publisher
        self.goal_pub = rospy.Publisher('swarm_goal', Pose, queue_size=1)

        # Subscribe
        rospy.Subscriber("swarm_goal_spd", Twist, self.update_swarm_goal)

        self.get_swarm_pose()

    # CF initialization
    def _init_cf(self, cf_id):
        """Initialize each CF

        Args:
            cf_id (str): Name of the CF

        Note:
            Dict for all the cf and their functions/parameters
            - Keys: cf_names
                - Dict:
                    - cf: Crazyflie instance
                    - emergency service
                    - goal_publisher
                    - get_pose
                    - initial_pose
        """
        self.crazyflies[cf_id] = {  "emergency": None,      # service  # TODO: add subdivision? i.e: self.crazyflies['cf1']['srv']['emergency]  
                                    "get_pose": None,       # service  # TODO: Or use a class?
                                    "take_off": None,       # service  
                                    "hover": None,          # service  
                                    "land": None,           # service  
                                    "stop": None,           # service
                                    "toggle_teleop": None,  # service  
                                    "set_pose": None,  # service  
                                    "initial_pose": None,   # attr
                                    "goal_msg": None,       # msg  
                                    "goal_pub": None}       # publisher

        # Subscribe to services
        if not to_sim:
            self._link_service(cf_id, "emergency", srv.Empty)

        self._link_service(cf_id, "get_pose", PoseRequest)
        self._link_service(cf_id, "take_off", srv.Empty)
        self._link_service(cf_id, "land", srv.Empty)
        self._link_service(cf_id, "hover", srv.Empty)
        self._link_service(cf_id, "stop", srv.Empty)
        self._link_service(cf_id, "toggle_teleop", srv.Empty)

        self.crazyflies[cf_id]["goal_msg"] = Pose()

        # Publish goal
        self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/' + cf_id + '/goal', Pose, queue_size=1)

        if self._to_sim:
            self._link_service(cf_id, "set_pose", PoseSet)
            self.crazyflies[cf_id]["initial_pose"] = self.formation.poses[cf_id]
            self.crazyflies[cf_id]["set_pose"](self.crazyflies[cf_id]["initial_pose"])

    def _link_service(self, cf_id, service_name, service_type):
        """Add a service to the dict of CFs

        Args:
            cf_id (str): Name of the CF
            service_name (str): Name of the serviec
            service_type (_): Type of the service
        """
        rospy.loginfo("Swarm: waiting for %s service of %s " % (service_name, cf_id))
        rospy.wait_for_service('/%s/%s' % (cf_id, service_name))
        rospy.loginfo("Swarm: found %s service of %s" % (service_name, cf_id))
        self.crazyflies[cf_id][service_name] = rospy.ServiceProxy('/%s/%s' % (cf_id, service_name), service_type)

    # Setter & getters
    def in_teleop(self):
        """
        Returns:
            bool: In teleop
        """
        return self._to_teleop
        
    # Services methods
    def toggleTeleop(self, req):
        """Toggle teleop mode

        Args:
            req (Empty): Empty

        Returns:
            EmptyResponse: Empty
        """
        self._to_teleop = not self._to_teleop
        self._call_all_cf_service("toggle_teleop")
        return srv.EmptyResponse()

    def update_params(self, req):
        """Update parameter of all swarm

        Args:
            req ([type]): [description]

        Returns:
            [type]: [description]
        """
        rospy.loginfo("Swarm: Update params")
        return srv.EmptyResponse()

    def emergency(self, req):
        """Call emergency service """
        rospy.logerr("Swarm: EMERGENCY")
        self._call_all_cf_service("emergency")
        return srv.EmptyResponse()

    def stop(self, req):
        """Call stop service """
        rospy.loginfo("Swarm: stop")
        self._call_all_cf_service("stop")
        return srv.EmptyResponse()
    
    def takeOff(self, req):
        rospy.loginfo("Swarm: take off")
        
        self.get_swarm_pose()

        goal_var = Twist()
        goal_var.linear.z = 0.5
        self.update_swarm_goal(goal_var)

        self._call_all_cf_service("take_off")
        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")

        goal_var = Twist()
        goal_var.linear.z = GND_HEIGHT - self.swarm_goal.position.z
        self.update_swarm_goal(goal_var)

        self._call_all_cf_service("land")
        return srv.EmptyResponse()
    
    def update_swarm_goal(self, goal_spd):
        """Update swarm_goal based on a change of speed

        Args:
            goal_spd (Twist): Speed variation of goal
        """
        self.formation.compute_goal(goal_spd)
        self.swarm_goal = self.formation.swarm_goal
        
        for cf_name, cf in self.crazyflies.items(): 
            cf["goal_msg"] =  self.formation.poses[cf_name]
 
    def get_swarm_pose(self):
        # TODO: #15 Initial swarm position depending on formation
        # To simplify, swarm pose is the average of all the poses

        x = []
        y = []
        z = []
        # yaw = []

        for _, cf in self.crazyflies.items(): 
            cf["initial_pose"] = cf["get_pose"]().pose
            cf["goal_msg"] = cf["initial_pose"]

            x.append(cf["initial_pose"].position.x)
            y.append(cf["initial_pose"].position.y)
            z.append(cf["initial_pose"].position.z)

        self.swarm_pose.position.x = np.mean(x)
        self.swarm_pose.position.y = np.mean(y)
        self.swarm_pose.position.z = np.mean(z)
        
        self.swarm_goal = self.swarm_pose

    def _call_all_cf_service(self, service_name, service_msg=None):
        """Call a service for all the CF in the swarm

        Args:
            service_name (str): Name of the service to call
            service_msg (srv_msg, optional): Message to send. Defaults to None.

        Returns:
            srv_res: Response of the service
        """
        for _, cf in self.crazyflies.items():
            if service_msg is None:
                res = cf[service_name]()
            else:
                res = cf[service_name](service_msg)
        return res

    def pub_goal(self):
        while not rospy.is_shutdown():
            self.goal_pub.publish(self.swarm_goal)

            for _, cf in self.crazyflies.items(): 
                cf["goal_pub"].publish(cf["goal_msg"])

            self.rate.sleep()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')

    # Get params
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    to_sim = rospy.get_param("~to_sim", "False")
    
    # Initialize swarm
    swarm = Swarm(cf_list, to_sim)

    swarm.pub_goal()

    rospy.spin()