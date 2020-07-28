joy_controller
==============

To control the swarm with a joystick.

Sends button pressed to :doc:`/ros_architecture/swarm_manager/swarm_api` through  ``swarm_manager/JoyButton`` service.


ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
/joy(`sensor_msgs/Joy`_)
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
/joy_button(swarm_manager/JoyButton)
    From :doc:`/ros_architecture/swarm_manager/swarm_api`

Parameters
^^^^^^^^^^
~joy_topic(str, default:"joy")

~sim(bool, default: False)

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

.. _sensor_msgs/Joy: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Joy.html
