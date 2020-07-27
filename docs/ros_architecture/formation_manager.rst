Formation Manager
=================

To manage the formation of the swarm

In charge of computing the positions of all the crazyflies.

Avalaible formations
---------------------
    - Square
    - Pyramid
    - Circle
    - V
    - Ligne

ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
:ref:`formation-goal-vel` (`geometry_msgs/Twist`_)
    Formation center goal variation

Published Topics
^^^^^^^^^^^^^^^^
:ref:`cf-formation-goal` (crazyflie_driver/Position)
    Goal of a single CF in the formation

:ref:`formation-goal` (crazyflie_driver/Position)
    Goal of formation

:ref:`formation-pose` (`geometry_msgs/Pose`_)
    Position of the formation


Services
^^^^^^^^
 /set_formation(formation_manager/SetFormation)
   Set swarm to a formation

 /formation_inc_scale(`std_srvs/Empty`_)
    Increase scale of formation

 /formation_dec_scale(`std_srvs/Empty`_)
    Decrease scale of formation

 /toggle_ctrl_mode(`std_srvs/Empty`_)
    To change between absolute and relative control mode

 /get_formations_list(formation_manager/GetFormationList)
    Return a list with all possible formations



Services Called
^^^^^^^^^^^^^^^
None

Parameters
^^^^^^^^^^
~n_cf(int)

.. _geometry_msgs/Twist: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
.. _geometry_msgs/Pose: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html