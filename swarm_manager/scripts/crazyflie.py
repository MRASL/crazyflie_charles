#!/usr/bin/env python

"""Class that represents a single crazyflie

Args:
    cf_name (str): Name of the crazyflie
    to_sim (bool): To run in simulation or not

:: _Voir les topics possibles:
    https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/crtp_commander_generic.c
"""

import rospy
import numpy as np

from crazyflie_driver.msg import Hover, Position
from crazyflie_driver.srv import UpdateParams
from std_msgs.msg import Empty as Empty_msg
from std_msgs.msg import String
from std_srvs.srv import Empty as Empty_srv
from geometry_msgs.msg import Twist, PoseStamped, Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from state_machine import StateMachine
from swarm_manager.srv import SetParam

class Crazyflie(object):
    """Controller of a single crazyflie.

    In charge of executing services and following positions

    Args:
        object ([type]): [description]
    """
    def __init__(self, cf_id, to_sim):
        """
        Args:
            cf_id (str): Name of the CF
            to_sim (bool): To sim

        Attributes:
            cf_id (str): Name of the CF
            _to_sim (bool): To sim

            _states (list of str): All possible states of the CF
            _state (str): Current state of the CF
        """
        # Attributes
        self.cf_id = '/' + cf_id

        self._to_sim = to_sim

        # Initialize state machine
        self._state_list = {"landed": self._stop,
                            "take_off":self._take_off,
                            "hover": self._hover,
                            "stop": self._stop,
                            "land": self._land,
                            "teleop": None}
        self._state_machine = StateMachine(self._state_list)
        self._state_machine.set_state("landed")

        rospy.loginfo("%s: Initializing" % self.cf_id)

        self.world_frame = rospy.get_param("~worldFrame", "/world")
        self.rate = rospy.Rate(10)

        # If not in simulation, find services and set parameters
        if not self._to_sim:
            rospy.loginfo(self.cf_id + ": waiting for update_params service...")
            rospy.wait_for_service(self.cf_id + '/update_params')
            rospy.loginfo(self.cf_id + ": found update_params service")
            self.update_param = rospy.ServiceProxy(self.cf_id + '/update_params', UpdateParams)

        # Declare services
        self._init_services()

        # Declare publishers
        self._init_publishers()

        # Declare subscriptions
        self.pose = Pose()
        self.goal = Position()
        self.initial_pose = Pose()

        rospy.Subscriber(self.cf_id + '/pose', PoseStamped, self._pose_handler)
        rospy.Subscriber(self.cf_id + '/goal', Position, self._goal_handler)

        # Find initial position
        self.localization_started = False #: boool: Sets to True upon receiveing first pose
        self.find_initial_pose()

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

        self.current_state_pub = rospy.Publisher(self.cf_id + "/state", String, queue_size=1)

    def _init_services(self):
        rospy.Service(self.cf_id + '/take_off', Empty_srv, self.take_off)
        rospy.Service(self.cf_id + '/hover', Empty_srv, self.hover)
        rospy.Service(self.cf_id + '/land', Empty_srv, self.land)
        rospy.Service(self.cf_id + '/stop', Empty_srv, self.stop)
        rospy.Service(self.cf_id + '/set_param', SetParam, self._set_param)

    # Handlers
    def _pose_handler(self, pose_stamped):
        """Update crazyflie position in world """
        self.localization_started = True
        self.pose = pose_stamped.pose

    def _goal_handler(self, goal):
        self.goal = goal

    def find_initial_pose(self):
        """ Find the initial position of the crazyflie.

        Position found by calculating the mean during a time interval
        """
        rate = rospy.Rate(100)
        while not self.localization_started:
            if rospy.is_shutdown():
                break

        initial_pose = {'x':[], 'y':[], 'z':[], 'yaw':[]}
        while len(initial_pose['x']) < 10:
            if rospy.is_shutdown():
                break

            initial_pose['x'].append(self.pose.position.x)
            initial_pose['y'].append(self.pose.position.y)
            initial_pose['z'].append(self.pose.position.z)
            initial_pose['yaw'].append(yaw_from_quat(self.pose.orientation))
            rate.sleep()

        self.initial_pose.position.x = np.mean(initial_pose['x'])
        self.initial_pose.position.y = np.mean(initial_pose['y'])
        self.initial_pose.position.z = np.mean(initial_pose['z'])
        self.initial_pose.orientation = quat_from_yaw(np.mean(initial_pose['yaw']))
        rospy.loginfo("%s: Initial position found" % self.cf_id)
        # rospy.loginfo(self.initial_pose)

    # Setter & Getters
    def _set_param(self, param_req):
        """Changes the value of the given parameter.

        Args:
            name (str): The parameter's name.
            value (Any): The parameter's value.
        """
        param_name = param_req.param
        param_val = param_req.value

        rospy.set_param(self.cf_id + "/" + param_name, param_val)

        if not self._to_sim:
            self.update_param([param_name])

        return {}

    def get_cf_id(self):
        """Get crazyflie id

        Returns:
            int: cf_id
        """
        return self.cf_id

    # Services
    def take_off(self, _):
        """Take off service
        """
        # rospy.loginfo("%s: Take off" % self.cf_id)
        self._state_machine.set_state("take_off")
        return {}

    def hover(self, _):
        """Hover service
        """
        # rospy.loginfo("%s: Hover" % self.cf_id)
        self._state_machine.set_state("hover")
        return {}

    def land(self, _):
        """Land service
        """
        # rospy.loginfo("%s: Landing" % self.cf_id)
        self._state_machine.set_state("land")
        return {}

    def stop(self, _):
        """Stop service
        """
        self._state_machine.set_state("stop")
        return {}

    # Methods depending on state
    def _take_off(self):
        z_dist = TAKE_OFF_HEIGHT
        x_start = self.pose.position.x
        y_start = self.pose.position.y
        z_start = self.pose.position.z
        yaw_start = yaw_from_quat(self.pose.orientation)

        duration = 3.0 # in sec
        n_steps = int(10*duration) # Where 10 is the frequency

        z_inc = z_dist/n_steps

        for i in range(n_steps):
            if rospy.is_shutdown() or not self._state_machine.in_state("take_off"):
                break

            new_z = i*z_inc + z_start

            self.cmd_pos(x_start, y_start, new_z, yaw_start)

            self.rate.sleep()

        if self._state_machine.in_state("take_off"):
            self._state_machine.set_state("hover")

    def _hover(self):
        self.cmd_pos_msg.header.seq += 1
        self.cmd_pos_msg.header.stamp = rospy.Time.now()

        self.cmd_pos(self.goal.x, self.goal.y, self.goal.z, self.goal.yaw)
        self.rate.sleep()

    def _land(self):
        rospy.sleep(0.2)

        x_start = self.pose.position.x
        y_start = self.pose.position.y
        z_start = self.pose.position.z
        yaw_start = yaw_from_quat(self.pose.orientation)

        z_dist = z_start - GND_HEIGHT

        time_range = 2*10

        z_dec = z_dist/time_range

        for i in range(time_range):
            if rospy.is_shutdown() or not self._state_machine.in_state("land"):
                break

            new_z = z_start - i*z_dec

            self.cmd_pos(x_start, y_start, new_z, yaw_start)

            self.rate.sleep()

        self.cmd_pos(x_start, y_start, GND_HEIGHT, yaw_start)
        self.rate.sleep()

        self._state_machine.set_state("stop")

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

    def cmd_hovering(self, z_distance):
        """Publish hover in cmd_hover topic

        Args:
            zDistance (float): Distance to hover
        """
        self.cmd_hovering_msg.zDistance = z_distance
        self.cmd_hovering_pub.publish(self.cmd_hovering_msg)

    def cmd_pos(self, x_val, y_val, _val, yaw):
        """Publish target position to cmd_positions topic

        Args:
            x_val (float): X
            y_val (float): Y
            z_val (float): Z
            yaw (float): Yaw
        """
        self.cmd_pos_msg.x = x_val
        self.cmd_pos_msg.y = y_val
        self.cmd_pos_msg.z = _val
        self.cmd_pos_msg.yaw = yaw
        self.cmd_pos_pub.publish(self.cmd_pos_msg)

    def publish_state(self):
        """Publish current state
        """
        self.current_state_pub.publish(self._state_machine.get_state())

    # Run methods
    def run(self):
        """Run controller
        """
        state_function = self._state_machine.run_state()
        state_function()

        self.publish_state()

def yaw_from_quat(quaternion):
    """Returns yaw from a quaternion

    Args:
        quaternion (Quaternion)

    Returns:
        float: Yaw
    """
    _, _, yaw = euler_from_quaternion([quaternion.x,
                                       quaternion.y,
                                       quaternion.z,
                                       quaternion.w])
    return yaw

def quat_from_yaw(yaw):
    """Compute a quaternion from yaw

    Pitch and roll are considered zero

    Args:
        yaw (float): Yaw

    Returns:
        Quaternion
    """
    x_quat, y_quat, z_quat, w_quat = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x_quat, y_quat, z_quat, w_quat)

if __name__ == '__main__':
    # Launch node
    rospy.init_node('cf_controller', anonymous=False)

    # Get params
    CF_ID = rospy.get_param("~cf_name", "cf_default")
    TO_SIM = rospy.get_param("~to_sim", "False")

    TAKE_OFF_HEIGHT = rospy.get_param("/swarm")["take_off_height"]
    GND_HEIGHT = rospy.get_param("/swarm")["gnd_height"]

    # Initialize cfx
    CF = Crazyflie(CF_ID, TO_SIM)

    while not rospy.is_shutdown():
        CF.run()
        