#!/usr/bin/env python

"""Generate rviz markers

Notes:
    Blue markers: Position of a CF
    Red: Goals of CF
    Green: Formation goal

:: _Rviz Markers wiki:
    http://wiki.ros.org/rviz/DisplayTypes/Marker
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from tf.transformations import quaternion_from_euler

WALL_HEIGHT = 1.0
WALL_THIC = 0.05
OBST_COL = [0.75, 0.55, 0.56, 0.57] # a, r, g, b

BED_POS = [2.30, 2.16, 0.1]
BED_SCALE = [1.5, 2.0, 0.2]

WALL_BOT_POS = [1.185, 0.0]
WALL_BOT_SCALE = [2.37, WALL_THIC]

WALL_LEFT_POS = [0.0, 1.58]
WALL_LEFT_SCALE = [WALL_THIC, 3.16]

WALL_TOP_POS = [1.525, 3.16]
WALL_TOP_SCALE = [3.05, WALL_THIC]

WALL_RIGHT_POS = [3.05, 1.905]
WALL_RIGHT_SCALE = [WALL_THIC, 2.45]

WALL_R_1_POS = [2.37, 0.34]
WALL_R_1_SCALE = [WALL_THIC, 0.68]

WALL_R_2_POS = [2.71, 0.68]
WALL_R_2_SCALE = [0.68, WALL_THIC]

class RvizMarkers(object):
    """To show CFs positions and obstacles in rviz
    """
    def __init__(self, cf_list):
        self.room_obstacles = init_room_msg()

        self.formation_goal_msg = init_formation_goal_msg()

        self.cf_pose_msgs = {}
        self.cf_goal_msgs = {}

        self.cf_list = cf_list

        self.rate = rospy.Rate(100)

        rospy.Subscriber("formation_goal", Position, self.update_formation_goal)

        for each_cf in cf_list:
            self.cf_pose_msgs[each_cf] = init_cf_pose_marker(each_cf)
            self.cf_goal_msgs[each_cf] = init_cf_goal_marker(each_cf)

            rospy.Subscriber("/%s/pose" % each_cf, PoseStamped, self.update_cf_pose, each_cf)
            rospy.Subscriber("/%s/goal" % each_cf, Position, self.update_cf_goal, each_cf)

        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)

    def update_cf_pose(self, pose_msg, cf_name):
        """Update position of a CF

        Args:
            pose_msg (PoseStamped): Current position
            cf_name (str): Name of CF
        """
        msg_sphere, msg_pose = self.cf_pose_msgs[cf_name]

        msg_sphere.pose.position = pose_msg.pose.position
        msg_sphere.pose.orientation = pose_msg.pose.orientation

        msg_pose.pose.position = pose_msg.pose.position
        msg_pose.pose.orientation = pose_msg.pose.orientation

    def update_cf_goal(self, goal_msg, cf_name):
        """Update goal of a CF

        Args:
            goal_msg (Position): Goal of the CF
            cf_name (str): Name of the CF
        """
        msg_sphere, msg_pose = self.cf_goal_msgs[cf_name]

        msg_sphere.pose.position.x = goal_msg.x
        msg_sphere.pose.position.y = goal_msg.y
        msg_sphere.pose.position.z = goal_msg.z
        msg_pose.pose.position = msg_sphere.pose.position

        [x_pose, y_pose, z_pose, w_pose] = quaternion_from_euler(0, 0, goal_msg.yaw)
        msg_pose.pose.orientation.x = x_pose
        msg_pose.pose.orientation.y = y_pose
        msg_pose.pose.orientation.z = z_pose
        msg_pose.pose.orientation.w = w_pose

    def update_formation_goal(self, goal):
        """Update formation goal

        Args:
            goal (Position): Formation goal
        """
        self.formation_goal_msg[0].pose.position.x = goal.x
        self.formation_goal_msg[0].pose.position.y = goal.y
        self.formation_goal_msg[0].pose.position.z = goal.z
        self.formation_goal_msg[1].pose.position = self.formation_goal_msg[0].pose.position

        [x_pose, y_pose, z_pose, w_pose] = quaternion_from_euler(0, 0, goal.yaw)
        self.formation_goal_msg[1].pose.orientation.x = x_pose
        self.formation_goal_msg[1].pose.orientation.y = y_pose
        self.formation_goal_msg[1].pose.orientation.z = z_pose
        self.formation_goal_msg[1].pose.orientation.w = w_pose

    def publish_obstacles(self):
        """Publish all obstacle markers
        """
        for each_obstacle in self.room_obstacles:
            self.marker_pub.publish(each_obstacle)

    def publish_formation_info(self):
        """Publish formation info
        """
        self.marker_pub.publish(self.formation_goal_msg[0])
        self.marker_pub.publish(self.formation_goal_msg[1])

    def publish_cf_info(self):
        """Publish all CFs info
        """
        # Publish all CFs pose
        for _, each_cf_msg in self.cf_pose_msgs.items():
            if rospy.is_shutdown():
                break
            self.marker_pub.publish(each_cf_msg[0]) # Pusblish sphere
            self.marker_pub.publish(each_cf_msg[1]) # Publish arrow

        # Publish all CFs goal
        for _, each_cf_msg in self.cf_goal_msgs.items():
            if rospy.is_shutdown():
                break
            self.marker_pub.publish(each_cf_msg[0]) # Pusblish sphere
            self.marker_pub.publish(each_cf_msg[1]) # Publish arrow

    def publish(self):
        """Publish all markers
        """
        while not rospy.is_shutdown():
            self.publish_formation_info()

            self.publish_cf_info()

            self.publish_obstacles()

            self.rate.sleep()

def init_room_msg():
    """Create obstacles in the room

    Returns:
        list of Marker: All obstacles in the room
    """
    obstacle_list = []

    obstacle_list.append(init_obstacle(0, BED_POS, BED_SCALE))
    obstacle_list.append(init_obstacle(1, WALL_BOT_POS, WALL_BOT_SCALE))
    obstacle_list.append(init_obstacle(2, WALL_LEFT_POS, WALL_LEFT_SCALE))
    obstacle_list.append(init_obstacle(3, WALL_RIGHT_POS, WALL_RIGHT_SCALE))
    obstacle_list.append(init_obstacle(4, WALL_R_1_POS, WALL_R_1_SCALE))
    obstacle_list.append(init_obstacle(5, WALL_R_2_POS, WALL_R_2_SCALE))
    obstacle_list.append(init_obstacle(6, WALL_TOP_POS, WALL_TOP_SCALE))


    return obstacle_list

def init_obstacle(obst_id, center_pos, scale):
    """Init obstacle at position

    If z isn't specified, sets to wall height

    Args:
        obst_id (int): obstacle id
        center_pos (list of float): Center position [x, y, z]
        scale (list of float): Wall length [x, y, z]

    Returns:
        Marker: Marker object
    """
    obst_msg = Marker()

    obst_msg.header.frame_id = "world"
    obst_msg.header.stamp = rospy.Time()

    obst_msg.ns = "obstacles"
    obst_msg.id = obst_id
    obst_msg.type = 1      # Cube
    obst_msg.action = 0    # Add

    obst_msg.pose.position.x = center_pos[0]
    obst_msg.pose.position.y = center_pos[1]

    if len(center_pos) == 3:
        obst_msg.pose.position.z = center_pos[2]
        obst_msg.scale.z = scale[2]
    else:
        obst_msg.pose.position.z = WALL_HEIGHT/2.0
        obst_msg.scale.z = WALL_HEIGHT

    obst_msg.pose.orientation.x = 0.0
    obst_msg.pose.orientation.y = 0.0
    obst_msg.pose.orientation.z = 0.0
    obst_msg.pose.orientation.w = 1.0

    obst_msg.scale.x = scale[0]
    obst_msg.scale.y = scale[1]

    obst_msg.color.a = OBST_COL[0]
    obst_msg.color.r = OBST_COL[1]
    obst_msg.color.g = OBST_COL[2]
    obst_msg.color.b = OBST_COL[3]

    return obst_msg

def init_formation_goal_msg():
    """Create formation goal msg

    Swarm goal is represented by a green sphere and arrow.

    Returns:
        [type]: [description]
    """
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time()
    msg.ns = "swarm"
    msg.id = 2
    msg.type = 2      # Sphere
    msg.action = 0    # Add
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.scale.x = 0.03
    msg.scale.y = 0.03
    msg.scale.z = 0.03
    msg.color.a = 0.8
    msg.color.r = 0
    msg.color.g = 1
    msg.color.b = 0

    msg_pose = Marker()
    msg_pose.header.frame_id = "world"
    msg_pose.header.stamp = rospy.Time()
    msg_pose.ns = "swarm"
    msg_pose.id = 3
    msg_pose.type = 0      # Arrow
    msg_pose.action = 0    # Add
    msg_pose.pose.position.x = 0
    msg_pose.pose.position.y = 0
    msg_pose.pose.position.z = 0
    msg_pose.pose.orientation.x = 0.0
    msg_pose.pose.orientation.y = 0.
    msg_pose.pose.orientation.z = 0.0
    msg_pose.pose.orientation.w = 1.0
    msg_pose.scale.x = 0.08
    msg_pose.scale.y = 0.01
    msg_pose.scale.z = 0.01
    msg_pose.color.a = 0.8
    msg_pose.color.r = 0
    msg_pose.color.g = 1
    msg_pose.color.b = 0

    return (msg, msg_pose)

def init_cf_pose_marker(cf_name):
    """Init msg for the position of a CF

    Position is shown as a blue oval with and arrow

    Args:
        cf_name (str): Name of CF

    Returns:
        list of Marker: Positon of the CF
    """
    marker_msg = Marker()

    marker_msg.header.frame_id = "world"
    marker_msg.header.stamp = rospy.Time()
    marker_msg.ns = cf_name
    marker_msg.id = 0
    marker_msg.type = 2      # Sphere
    marker_msg.action = 0    # Add
    marker_msg.pose.position.x = 0
    marker_msg.pose.position.y = 0
    marker_msg.pose.position.z = 0
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.1
    marker_msg.scale.y = 0.1
    marker_msg.scale.z = 0.05
    marker_msg.color.a = 1
    marker_msg.color.r = 0
    marker_msg.color.g = 0
    marker_msg.color.b = 1

    pose_msg = Marker()
    pose_msg.header.frame_id = "world"
    pose_msg.header.stamp = rospy.Time()
    pose_msg.ns = cf_name
    pose_msg.id = 1
    pose_msg.type = 0      # Arrow
    pose_msg.action = 0    # Add
    pose_msg.pose.position.x = 0
    pose_msg.pose.position.y = 0
    pose_msg.pose.position.z = 0
    pose_msg.pose.orientation.x = 0.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    pose_msg.pose.orientation.w = 1.0
    pose_msg.scale.x = 0.07
    pose_msg.scale.y = 0.01
    pose_msg.scale.z = 0.01
    pose_msg.color.a = 0.8
    pose_msg.color.r = 0
    pose_msg.color.g = 0
    pose_msg.color.b = 1

    return [marker_msg, pose_msg]

def init_cf_goal_marker(cf_name):
    """Init msg for the goal of a CF

    Goal is shown as a red oval with and arrow

    Args:
        cf_name (str): Name of CF

    Returns:
        list of Marker: Goal of the CF
    """
    marker_msg = Marker()

    marker_msg.header.frame_id = "world"
    marker_msg.header.stamp = rospy.Time()
    marker_msg.ns = cf_name
    marker_msg.id = 2
    marker_msg.type = 2      # Sphere
    marker_msg.action = 0    # Add
    marker_msg.pose.position.x = 0
    marker_msg.pose.position.y = 0
    marker_msg.pose.position.z = 0
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.03
    marker_msg.scale.y = 0.03
    marker_msg.scale.z = 0.03
    marker_msg.color.a = 0.7
    marker_msg.color.r = 1
    marker_msg.color.g = 0
    marker_msg.color.b = 0

    pose_msg = Marker()
    pose_msg.header.frame_id = "world"
    pose_msg.header.stamp = rospy.Time()
    pose_msg.ns = cf_name
    pose_msg.id = 3
    pose_msg.type = 0      # Arrow
    pose_msg.action = 0    # Add
    pose_msg.pose.position.x = 0
    pose_msg.pose.position.y = 0
    pose_msg.pose.position.z = 0
    pose_msg.pose.orientation.x = 0.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = 0.0
    pose_msg.pose.orientation.w = 1.0
    pose_msg.scale.x = 0.05
    pose_msg.scale.y = 0.01
    pose_msg.scale.z = 0.01
    pose_msg.color.a = 1
    pose_msg.color.r = 1
    pose_msg.color.g = 0
    pose_msg.color.b = 0

    return [marker_msg, pose_msg]

if __name__ == '__main__':
    rospy.init_node('rviz_markers', anonymous=False)

    CF_LIST = rospy.get_param("~cf_list", "['cf1']")
    RVIZ_MARKERS = RvizMarkers(CF_LIST)

    RVIZ_MARKERS.publish()
