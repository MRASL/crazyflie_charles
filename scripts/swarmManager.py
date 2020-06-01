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
from crazyflie import Crazyflie
from crazyflie_sim import CrazyflieSim

from geometry_msgs.msg import Pose
from std_srvs import srv
from crazyflie_charles.srv import PoseRequest

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
        rospy.Service('/getSwarmPose', PoseRequest, self.get_swarm_pose)    # Find position of the swarm

        # Subscribe
        rospy.Subscriber("goal_swarm", Pose, self.broadcast_goal)

        self._to_teleop = False   

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
                                    "initial_pose": None,   # attribute  
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

        # Publish goal
        self.crazyflies[cf_id]["goal_pub"] = rospy.Publisher('/' + cf_id + '/goal', Pose, queue_size=1)

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
        self._call_all_cf_service("take_off")
        return srv.EmptyResponse()
    
    def land(self, req):
        rospy.loginfo("Swarm: land")
        self._call_all_cf_service("land")
        return srv.EmptyResponse()
    
    def broadcast_goal(self, goal):
        # TODO: Different goal for each CF depending on formation (#15)
        for _, cf in self.crazyflies.items(): 
            cf["goal_pub"].publish(goal)

    def get_swarm_pose(self, req):
        swarmPose = Pose()
        for _, cf in self.crazyflies.items(): 
            cf["initial_pose"] = cf["get_pose"]()
            swarmPose = cf["initial_pose"]
            
        # TODO: #15 Initial swarm position depending on formation
        # For one CF swarm pos = CF pos
        return swarmPose

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