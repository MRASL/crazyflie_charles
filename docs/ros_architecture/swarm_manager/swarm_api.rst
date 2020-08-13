SwarmAPI
========

Node to connect python API to ros.

ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
Nones

Published Topics
^^^^^^^^^^^^^^^^
None

Services
^^^^^^^^
/joy_button(swarm_manager/JoyButton)
    To get button pressed on joystick

Services Called
^^^^^^^^^^^^^^^
/swarm_emergency(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/stop_swarm(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/take_off_swarm(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/land_swarm(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/set_mode(swarm_manager/SetMode)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/go_to(swarm_manager/SetGoals)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/get_positions(swarm_manager/GetPositions)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/set_swarm_formation(formation_manager/SetFormation)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/next_swarm_formation(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/prev_swarm_formation(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/inc_swarm_scale(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/dec_swarm_scale(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`

/toggle_ctrl_mode(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`


Parameters
^^^^^^^^^^
None

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
