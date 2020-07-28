cf_controller
=============

Node to control a single crazyflie. Each CF has it's own node.

ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
:ref:`cf-goal` (crazyflie_driver/Position)
    Goal of crazyflie

:ref:`cf-pose` (geometry_msgs/PoseStamped)
    Current pose of CF



Published Topics
^^^^^^^^^^^^^^^^
:ref:`cf-state` (std_msgs/String)
    Current state of CF

:ref:`cmd-position` (crazyflie_drive/Position)
    Move CF to absolute position

:ref:`cmd-hovering` (crazyflie_driver/Hover)
    Hover CF

:ref:`cmd-vel` (`geometry_msgs/Twist`_)
    Control velocity of CF

:ref:`cmd-stop` (`std_msgs/Empty`_)
    Stop CF

Services
^^^^^^^^
 /take_off(`std_srvs/Empty`_)
    Take off CF

 /hover(`std_srvs/Empty`_)
    Hover CF

 /land(`std_srvs/Empty`_)
    Land CF

 /stop(`std_srvs/Empty`_)
    Stop CF

 /set_param (swarm_manager/SetParam)
    Set a parameter


Services Called
^^^^^^^^^^^^^^^
None

Parameters
^^^^^^^^^^
~cf_name(str, default: cf_default)

~sim(bool, default: False)

~take_off_height(float)

~gnd_height(float)

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
.. _geometry_msgs/Twist: http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
.. _std_msgs/Empty: http://docs.ros.org/melodic/api/std_msgs/html/msg/Empty.html