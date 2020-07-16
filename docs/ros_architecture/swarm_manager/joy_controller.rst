joy_controller
==============

To control the swarm with a joystick.

.. todo:: Joystick axis and button mapping

Usage
-----


ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
/joy(sensor_msgs/Joy)
    Joystick input


Published Topics
^^^^^^^^^^^^^^^^
:ref:`joy-swarm-vel` (geometry_msgs/Twist)
    Swarm velocity


Services
^^^^^^^^
None

Services Called
^^^^^^^^^^^^^^^
/swarm_emergency(`std_srvs/Empty`_)

/toggle_teleop(`std_srvs/Empty`_)

/land_swarm(`std_srvs/Empty`_)

/take_off_swarm(`std_srvs/Empty`_)

/stop_swarm(`std_srvs/Empty`_)

/inc_swarm_scale(`std_srvs/Empty`_)

/dec_swarm_scale(`std_srvs/Empty`_)

/toggle_ctrl_mode(`std_srvs/Empty`_)

/next_swarm_formation(`std_srvs/Empty`_)

/prev_swarm_formation(`std_srvs/Empty`_)


Parameters
^^^^^^^^^^
~joy_topic(str, default:"joy")

~to_sim(bool, default: False)

~teleop(bool, default: False)

~x_axis(int, default: 4)

~y_axis(int, default: 3)

~z_axis(int, default: 2)

~yaw_axis(int, default: 1)

~x_velocity_max(float, default: 2.0)

~y_velocity_max(float, default: 2.0)

~z_velocity_max(float, default: 2.0)

~yaw_velocity_max(float, default: 2.0)

~x_goal_max(float, default: 0.05)

~y_goal_max(float, default: 0.05)

~z_goal_max(float, default: 0.05)

~yaw_goal_max(float, default: 0.05)

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
