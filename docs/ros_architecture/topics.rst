ROS Topics
==========

Description of the ros topics used.

+-----------------------+-------------------------------+---------------------------+
| Topic                 | Msg type                      | Description               |
+=======================+===============================+===========================+
|/cfx/pose              |(geometry_msgs/PoseStamped)    | Pose of a CF              |
+-----------------------+-------------------------------+---------------------------+
|/cfx/state             |(std_msgs/String)              | State of a CF             |
+-----------------------+-------------------------------+---------------------------+
|/cfx/cmd_position      |(crazyflie_drive/Position)     | Position command          |
+-----------------------+-------------------------------+---------------------------+
|/cfx/cmd_hovering      |(crazyflie_driver/Hover)       | Hovering command          |
+-----------------------+-------------------------------+---------------------------+
|/cfx/cmd_vel           |(geometry_msgs/Twist)          | Velocity of CF            |
+-----------------------+-------------------------------+---------------------------+
|/cfx/cmd_stop          |(std_msgs/Empty)               | Stop CF                   |
+-----------------------+-------------------------------+---------------------------+
|/cfx/trajectory_goal   |(crazyflie_driver/Position)    | Trajectory goal of a CF   |
+-----------------------+-------------------------------+---------------------------+
|/cfx/formation_goal    |(crazyflie_driver/Position)    | Formation goal of a CF    |
+-----------------------+-------------------------------+---------------------------+
|/formation_goal        |(crazyflie_drive/Position)     | Goal of formation         |
+-----------------------+-------------------------------+---------------------------+
|/formation_pose        |(geometry_msgs/Pose)           | Position of formation     |
+-----------------------+-------------------------------+---------------------------+
|/formation_goal_vel    |(geometry_msgs/Twist)          | Formation goal velocity   |
+-----------------------+-------------------------------+---------------------------+
|/joy_swarm_vel         |(geometry_msgs/Twist)          | Joystick swarm command    |
+-----------------------+-------------------------------+---------------------------+



Crazyflie
---------

.. _cf-pose:

/cfx/pose
^^^^^^^^^
    (geometry_msgs/PoseStamped)

    Current pose of CF


.. _cf-state:

/cfx/state
^^^^^^^^^^
    (std_msgs/String)

    Current state of CF

.. _cf-goal:

/cfx/goal
^^^^^^^^^
    (crazyflie_driver/Position)

    Target position of CF

.. _cmd-position:

/cfx/cmd_position
^^^^^^^^^^^^^^^^^
    (crazyflie_drive/Position)

    Position command

.. _cmd-hovering:

/cfx/cmd_hovering
^^^^^^^^^^^^^^^^^
    (crazyflie_driver/Hover)

    Hovering command

.. _cmd-vel:

/cfx/cmd_vel
^^^^^^^^^^^^
    (geometry_msgs/Twist)

    Velocity of CF

.. _cmd-stop:

/cfx/cmd_stop
^^^^^^^^^^^^^
    (std_msgs/Empty)

    Stop CF

Swarm
-----

.. _trajectory-goal:

/cfx/trajectory_goal
^^^^^^^^^^^^^^^^^^^^
    (crazyflie_driver/Position)

    Position of the CF on the trajectory, at each time step


.. _cf-formation-goal:

/cfx/formation_goal
^^^^^^^^^^^^^^^^^^^^
    (crazyflie_driver/Position)

    Goal of a single CF in the formation

.. _formation-goal:

/formation_goal
^^^^^^^^^^^^^^^
    (crazyflie_driver/Position)

    Goal of formation

.. _formation-pose:

/formation_pose
^^^^^^^^^^^^^^^
    (geometry_msgs/Pose)

    Position of the formation

.. _formation-goal-vel:

/formation_goal_vel
^^^^^^^^^^^^^^^^^^^
    (geometry_msgs/Twist)

    Formation center goal variation

.. _joy-swarm-vel:

/joy_swarm_vel
^^^^^^^^^^^^^^
    (geometry_msgs/Twist)

    Swarm velocity, from joystick