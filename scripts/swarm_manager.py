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

from geometry_msgs.msg import Pose, Twist, Quaternion
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest, PoseSet, SetFormation
from crazyflie_driver.msg import Position

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

        self.swarm_goal = Position()
        self.swarm_goal_vel = Twist()

        # Initialize each Crazyflie
        for each_cf in cf_list:
            self._init_cf(each_cf)

        # Services
        rospy.Service('/update_params', srv.Empty, self.update_params)      # TODO: #14 Update all parameters
        rospy.Service('/emergency', srv.Empty, self.emergency)              # Emergency
        rospy.Service('/stop', srv.Empty, self.stop)                        # Stop all CFs
        rospy.Service('/take_off', srv.Empty, self.take_off)                  # Take off all CFs
        rospy.Service('/land', srv.Empty, self.land)                        # Land all CFs
        rospy.Service('/toggle_teleop', srv.Empty, self.toggle_teleop)        # Toggle between manual and auto mode

        # Formation services
        rospy.loginfo("Swarm: waiting for services")
        # rospy.loginfo("Swarm: waiting for %s service" % "set_formation")
        rospy.wait_for_service("set_formation")
        # rospy.loginfo("Swarm: found %s service" % "set_formation")
        self.set_formation = rospy.ServiceProxy("set_formation", SetFormation)

        # rospy.loginfo("Swarm: waiting for %s service" % "update_swarm_goal")
        rospy.wait_for_service("update_swarm_goal")
        # rospy.loginfo("Swarm: found %s service" % "update_swarm_goal")
        self.update_swarm_goal = rospy.ServiceProxy("update_swarm_goal", srv.Empty)
        rospy.loginfo("Swarm: services found")

        # Publisher
        self.goal_pub = rospy.Publisher('swarm_goal', Position, queue_size=1)
        self.goal_vel_pub = rospy.Publisher('swarm_goal_vel', Twist, queue_size=1)

        # Subscribe
        rospy.Subscriber("swarm_goal", Position, self.swarm_goal_handler)
        rospy.Subscriber("swarm_goal_vel", Twist, self.swarm_goal_vel_handler)

        self.formation = "square"
        self.set_formation(self.formation)

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
                                    "set_pose": None,       # service  
                                    "initial_pose": None,   # attr
                                    "goal_msg": None,       # msg  
                                    "goal_pub": None}       # publisher

        # Subscribe to services
        if not to_sim:
            self._link_service(cf_id, "emergency", srv.Empty)

        else:
            pass
            # self._link_service(cf_id, "set_pose", PoseSet)
            # self.crazyflies[cf_id]["initial_pose"] = self.formation.cf_goals[cf_id]
            # self.crazyflies[cf_id]["set_pose"](self.crazyflies[cf_id]["initial_pose"])

        rospy.loginfo("Swarm: waiting services of %s " % cf_id)
        self._link_service(cf_id, "get_pose", PoseRequest)
        self._link_service(cf_id, "take_off", srv.Empty)
        self._link_service(cf_id, "land", srv.Empty)
        self._link_service(cf_id, "hover", srv.Empty)
        self._link_service(cf_id, "stop", srv.Empty)
        self._link_service(cf_id, "toggle_teleop", srv.Empty)
        rospy.loginfo("Swarm: found services of %s " % cf_id)

        # CF goal
        # TODO Enlever?
        self.crazyflies[cf_id]["goal_msg"] = Position()
        self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/' + cf_id + '/goal', Position, queue_size=1)

    def _link_service(self, cf_id, service_name, service_type):
        """Add a service to the dict of CFs

        Args:
            cf_id (str): Name of the CF
            service_name (str): Name of the serviec
            service_type (_): Type of the service
        """
        # rospy.loginfo("Swarm: waiting for %s service of %s " % (service_name, cf_id))
        rospy.wait_for_service('/%s/%s' % (cf_id, service_name))
        # rospy.loginfo("Swarm: found %s service of %s" % (service_name, cf_id))
        self.crazyflies[cf_id][service_name] = rospy.ServiceProxy('/%s/%s' % (cf_id, service_name), service_type)

    # Setter & getters
    def in_teleop(self):
        """
        Returns:
            bool: In teleop
        """
        return self._to_teleop
    
    # Publisher and subscription\
    def pub_goal(self):
        while not rospy.is_shutdown():
            # self.goal_pub.publish(self.swarm_goal)
            self.rate.sleep()

    def pub_swarm_goal_vel(self, goal_spd):
        """Publish new swarm_goal speed

        Args:
            goal_spd (Twist): Speed variation of goal
        """
        self.goal_vel_pub.publish(goal_spd)

    def swarm_goal_handler(self, swarm_goal):
        self.swarm_goal = swarm_goal

    def swarm_goal_vel_handler(self, swarm_goal_vel):
        self.swarm_goal_vel = swarm_goal_vel

    # Services methods
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

    def toggle_teleop(self, req):
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
    
    def take_off(self, req):
        rospy.loginfo("Swarm: take off")

        self.update_swarm_goal() 
        goal_var = Twist()
        goal_var.linear.z = 0.5
        self.pub_swarm_goal_vel(goal_var)

        self._call_all_cf_service("take_off")
        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")

        self.update_swarm_goal()
        goal_var = Twist()
        goal_var.linear.z = GND_HEIGHT - self.swarm_goal.z
        self.pub_swarm_goal_vel(goal_var)

        self._call_all_cf_service("land")
        return srv.EmptyResponse()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('swarm_manager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')

    # Get params
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    to_sim = rospy.get_param("~to_sim", "False")
    
    # Initialize swarm
    swarm = Swarm(cf_list, to_sim)

    swarm.pub_goal()

    rospy.spin()