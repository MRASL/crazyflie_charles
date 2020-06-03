#!/usr/bin/env python

"""Class that represents a single crazyflie

Args:
    cf_name (str): Name of the crazyflie
    to_sim (bool): To run in simulation or not

:: _Voir les topics possibles: 
    https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c
"""

import rospy
import tf
from tf.transformations import euler_from_quaternion
import numpy as np

from crazyflie_driver.msg import Hover, Position
from std_msgs.msg import Empty as Empty_msg
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse as EmptyResponse_srv
from crazyflie_driver.srv import UpdateParams
from geometry_msgs.msg import Twist, PoseStamped, Pose
from crazyflie_charles.srv import PoseRequest

class Crazyflie:
    def __init__(self, cf_id, to_sim):
        """
        Args:
            cf_id (str): Name of the CF
            to_sim (bool): To sim

        Attributes:
            cf_id (str): Name of the CF
            _to_sim (bool): To sim
            _to_teleop (bool): To teleop
            
            _states (list of str): All possible states of the CF
            _state (str): Current state of the CF

        Publishers:

        Messages:

        Services:


        """
        # Attributes
        self.cf_id = '/' + cf_id

        self._to_sim = to_sim
        self._to_teleop = False

        self._states = ["take_off", "land", "hover", "stop", "teleop"]
        self._state = "stop"
        self._setState("stop")
        
        rospy.loginfo("%s: Initializing" % self.cf_id)
        

        self.world_frame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        # If not in simulation, find services and set parameters
        if not self._to_sim:
            rospy.loginfo(self.cf_id + ": waiting for update_params service...")
            rospy.wait_for_service(self.cf_id + '/update_params')
            rospy.loginfo(self.cf_id + ": found update_params service")
            self.update_params = rospy.ServiceProxy(self.cf_id + '/update_params', UpdateParams)

            # Set parameters # TODO: Move to swarmManager
            self.setParam("kalman/resetEstimation", 1)     

        # Declare services
        self._init_services()

        # Declare publishers
        self._init_publishers()

        # Declare subscriptions
        self.pose = Pose()
        self.goal = Position()
        self.initial_pose = Pose()
        self.findInitialPose()

        rospy.Subscriber(self.cf_id + '/pose', PoseStamped, self._pose_handler)
        rospy.Subscriber(self.cf_id + '/goal', Position, self._goal_handler)

        rospy.loginfo("%s: Setup done" % self.cf_id)

    def _init_publishers(self):
        """Initialize all publishers"""
        self.cmd_vel_pub = rospy.Publisher(self.cf_id + '/cmd_vel', Twist, queue_size=1)
        self.cmd_vel_msg = Twist()

        self.cmd_hovering_pub = rospy.Publisher(self.cf_id + "/cmd_hovering", Hover, queue_size=1)
        self.cmd_hovering_msg = Hover()
        self.cmd_hovering_msg.header.seq = 0
        self.cmd_hovering_msg.header.stamp = rospy.Time.now()
        self.cmd_hovering_msg.header.frame_id = self.world_frame
        self.cmd_hovering_msg.yawrate = 0
        self.cmd_hovering_msg.vx = 0
        self.cmd_hovering_msg.vy = 0

        self.cmd_pos_pub = rospy.Publisher(self.cf_id + "/cmd_position", Position, queue_size=1)
        self.cmd_pos_msg = Position()
        self.cmd_pos_msg.header.seq = 1
        self.cmd_pos_msg.header.stamp = rospy.Time.now()
        self.cmd_pos_msg.header.frame_id = self.world_frame
        self.cmd_pos_msg.yaw = 0

        self.cmd_stop_pub = rospy.Publisher(self.cf_id + "/cmd_stop", Empty_msg, queue_size=1)
        self.cmd_stop_msg = Empty_msg()
    
    def _init_services(self):
        rospy.Service(self.cf_id + '/get_pose', PoseRequest, self.returnPose)
        rospy.Service(self.cf_id + '/take_off', Empty_srv, self.take_off)
        rospy.Service(self.cf_id + '/hover', Empty_srv, self.hover)
        rospy.Service(self.cf_id + '/land', Empty_srv, self.land)
        rospy.Service(self.cf_id + '/stop', Empty_srv, self.stop)
        rospy.Service(self.cf_id + '/toggle_teleop', Empty_srv, self.toggleTeleop)

    # Handlers
    def _pose_handler(self, pose_stamped):
        """Update crazyflie position in world """
        self.pose = pose_stamped.pose

    def _goal_handler(self, goal):
        self.goal = goal

    def returnPose(self, req):
        self.findInitialPose()

        return self.initial_pose

    def findInitialPose(self):  
        """ Find the initial position of the crazyflie by calculating the mean during a time interval"""
        if not self._to_sim:
            r = rospy.Rate(100)
            initialPose = {'x': [], 'y':[], 'z':[] } 
            while len(initialPose['x']) < 10:
                initialPose['x'].append(self.pose.position.x)
                initialPose['y'].append(self.pose.position.y)
                initialPose['z'].append(self.pose.position.z)
                r.sleep()
            
            self.initial_pose.position.x = np.mean(initialPose['x'])
            self.initial_pose.position.y = np.mean(initialPose['y'])
            self.initial_pose.position.z = np.mean(initialPose['z'])
            rospy.loginfo("Initial position: \n{}".format(self.initial_pose))

        else:
            self.initial_pose = self.pose

    # Setter & Getters
    def setParam(self, name, value):
        """Changes the value of the given parameter.

        Args:
            name (str): The parameter's name.
            value (Any): The parameter's value.
        """

        rospy.set_param(self.cf_id + "/" + name, value)
        self.update_params([name])

    def getId(self):
        return self.cf_id

    def in_teleop(self):
        return self._state == "teleop"

    # State manager
    def _setState(self, newState):
        if newState in self._states:
            self._state = newState
        else:
            rospy.logerr("Invalid State: %s" % newState)

    def take_off(self, req):
        # rospy.loginfo("%s: Take off" % self.cf_id)
        self._setState("take_off")
        return EmptyResponse_srv()

    def hover(self, req):
        # rospy.loginfo("%s: Hover" % self.cf_id)
        self._setState("hover")
        return EmptyResponse_srv()

    def land(self, req):
        # rospy.loginfo("%s: Landing" % self.cf_id)
        self._setState("land")
        return EmptyResponse_srv()

    def stop(self, req):
        # rospy.loginfo("%s: Stoping" % self.cf_id)
        self._setState("stop")
        return EmptyResponse_srv()

    def toggleTeleop(self, req):
        if self._state == "teleop":
            self._setState("stop")
        else:
            self._setState("teleop")

        return EmptyResponse_srv()

    # Methods depending on state
    def _take_off(self):
        dZ = self.goal.z - self.pose.position.z
        x_start = self.pose.position.x 
        y_start = self.pose.position.y
        z_start = self.pose.position.z

        # rospy.loginfo("Going to \n{}".format(self.goal))

        time_range = 1*10
        z_inc = dZ/time_range
        
        
        for i in range(time_range):
            if rospy.is_shutdown() or self._state is not "take_off": break

            z = i*z_inc + z_start

            self.cmd_pos(x_start, y_start, z, self.goal.yaw)

            self.rate.sleep()

        if self._state is "take_off":
            self.hover(Empty_srv())

    def _hover(self):
        self.cmd_pos_msg.header.seq += 1
        self.cmd_pos_msg.header.stamp = rospy.Time.now()

        self.cmd_pos(self.goal.x, self.goal.y, self.goal.z, self.goal.yaw)
        self.rate.sleep()

    def _land(self):
        x_start = self.pose.position.x
        y_start = self.pose.position.y
        z_start = self.pose.position.z

        dZ =  z_start - self.goal.z


        time_range = 2*10

        z_dec = dZ/time_range
        
        for i in range(time_range):
            if rospy.is_shutdown() or self._state is not "land": break

            z = z_start - i*z_dec 

            self.cmd_pos(x_start, y_start, z, 0)

            self.rate.sleep()

        self.cmd_pos(self.goal.x, self.goal.y, self.goal.z, self.goal.yaw)
        self.rate.sleep()

        self.stop(Empty_srv())

    def _stop(self):
        self.cmd_vel(0, 0, 0, 0)
        self.rate.sleep()

    # Publishing methods
    def cmd_vel(self, roll, pitch, yawrate, thrust):
        """Publish pose in cmd_vel topic

        Args:
            roll (float): Roll angle. Degrees. Positive values == roll right.
            pitch (float): Pitch angle. Degrees. Positive values == pitch
                forward/down.
            yawrate (float): Yaw angular velocity. Degrees / second. Positive
                values == turn counterclockwise.
            thrust (float): Thrust magnitude. Non-meaningful units in [0, 2^16),
                where the maximum value corresponds to maximum thrust.
        """
        msg = Twist()
        msg.linear.x = pitch
        msg.linear.y = roll
        msg.angular.z = yawrate
        msg.linear.z = thrust
        self.cmd_vel_pub.publish(msg)

    def cmd_hovering(self, zDistance):
        """Publish hover in cmd_hover topic

        Args:
            zDistance (float): Distance to hover
        """
        self.cmd_hovering_msg.zDistance = zDistance
        self.cmd_hovering_pub.publish(self.cmd_hovering_msg)

    def cmd_pos(self, x, y, z, yaw):
        """Publish target position to cmd_positions topic

        Args:
            x (float): X
            y (float): Y
            z (float): Z
            yaw (float): Yaw
        """
        self.cmd_pos_msg.x = x
        self.cmd_pos_msg.y = y
        self.cmd_pos_msg.z = z
        self.cmd_pos_msg.yaw = yaw
        self.cmd_pos_pub.publish(self.cmd_pos_msg)

    # Run methods
    def run_auto(self):
        if self._state == "take_off":
            self._take_off()
        elif self._state == "hover":
            self._hover()
        elif self._state == "land":
            self._land()
        else:
            self._stop()

if __name__ == '__main__':
    # Launch node
    rospy.init_node('cf_controller', anonymous=False)

    # Get params
    cf_id = rospy.get_param("~cf_name", "cf_default")
    to_sim = rospy.get_param("~to_sim", "False")
    
    # Initialize cfx
    cf = Crazyflie(cf_id, to_sim)

    while not rospy.is_shutdown():
        if not cf.in_teleop():
            cf.run_auto()

        else:
            pass