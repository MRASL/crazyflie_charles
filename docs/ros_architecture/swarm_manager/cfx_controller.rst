cfx_controller node
===================

Node to control a single crazyflie. Each CF has it's own node.


Usage
-----


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

:ref:`cmd-vel` (geometry_msgs/Twist)
    Control velocity of CF

:ref:`cmd-stop` (std_msgs/Empty)
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


Services Called
^^^^^^^^^^^^^^^
None

Parameters
^^^^^^^^^^
~cf_name(str, default: cf_default)
~to_sim(bool, default: False)
~take_off_height(float)
~gnd_height(float)

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
