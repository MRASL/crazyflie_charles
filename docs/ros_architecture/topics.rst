ROS Topics
==========

Description of the ros topics used.

Crazyflie
---------

.. _cf-pose:

/cfx/pose
^^^^^^^^^
    (geometry_msgs/PoseStamped)


.. _cf-state:

/cfx/state
^^^^^^^^^^
    (std_msgs/String)


.. _cf-goal:

/cfx/goal
^^^^^^^^^
    (crazyflie_driver/Position)


.. _cmd-position:

/cfx/cmd_position
^^^^^^^^^^^^^^^^^
    crazyflie_drive/Position


.. _cmd-hovering:

/cfx/cmd_hovering
^^^^^^^^^^^^^^^^^
    crazyflie_driver/Hover


.. _cmd-vel:

/cfx/cmd_vel
^^^^^^^^^^^^
    geometry_msgs/Twist


.. _cmd-stop:

/cfx/cmd_stop
^^^^^^^^^^^^^
    std_msgs/Empty


Swarm
-----

.. _trajectory-goal:

/cfx/trajectory_goal
^^^^^^^^^^^^^^^^^^^^
    (crazyflie_driver/Position)


.. _formation-goal:

/cfx/formation_goal
^^^^^^^^^^^^^^^^^^^^
    (crazyflie_driver/Position)


.. _joy-swarm-vel:

/joy_swarm_vel
^^^^^^^^^^^^^^
    (geometry_msgs/Twist)


.. _formation-goal-vel:

/formation_goal_vel
^^^^^^^^^^^^^^^^^^^
    (geometry_msgs/Twist)