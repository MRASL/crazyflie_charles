swarm_controller
================

To control flight of the swarm. By using a state machine, this node will determine each crazyflie goal and
control it's desired position by publishing to :ref:`cf-goal`.

The goal will change depending of the services called by SwarmAPI.

.. note::
   See :doc:`/glossary` for definition of Swarm and Formation


ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
:ref:`cf-formation-goal` (crazyflie_driver/Position)
    Position of the CF in formation

:ref:`trajectory-goal` (crazyflie_driver/Position)
    Position of the CF on the trajectory, at each time step

:ref:`cf-state` (std_msgs/String)
    Current state of CF

:ref:`cf-pose` (geometry_msgs/PoseStamped)
    Current pose of CF

:ref:`joy-swarm-vel` (geometry_msgs/Twist)
    Swarm velocity


Published Topics
^^^^^^^^^^^^^^^^
:ref:`cf-goal` (crazyflie_driver/Position)
    Target position of CF

:ref:`formation-goal-vel` (geometry_msgs/Twist)
    Formation center goal variation

Services
^^^^^^^^
 /take_off_swarm(`std_srvs/Empty`_)
    Take off all CFs

 /stop_swarm(`std_srvs/Empty`_)
    Stop all CFs

 /swarm_emergency(`std_srvs/Empty`_)
    Emgergency stop of all CFs

 /land_swarm(`std_srvs/Empty`_)
    Land all CF to their starting position

 /get_positions(swarm_manager/GetPositions)
    Get current position of CFs

 /go_to(swarm_manager/SetGoals)
    Move CFs or formation to specified positions

 /set_mode(swarm_manger/SetMode)
   Set control of swarm_controller
 
 /set_swarm_formation(formation_manager/SetFormation)
   Set swarm to a formation

 /inc_swarm_scale(`std_srvs/Empty`_)
    Increase scale of formation

 /dec_swarm_scale(`std_srvs/Empty`_)
    Decrease scale of formation

 /next_swarm_formation(`std_srvs/Empty`_)
    Go to next formation

 /prev_swarm_formation(`std_srvs/Empty`_)
    Go to previous formation

 /traj_found(`std_srvs/SetBool`_)
    To call once the trajectory planner is done

 /traj_done(`std_srvs/Empty`_)
    To call once the trajectory is done

Services Called
^^^^^^^^^^^^^^^
 /set_formation(formation_manager/SetFormation)
    From :doc:`/ros_architecture/formation_manager`

 /get_formations_list(formation_manager/GetFormationList)
    From :doc:`/ros_architecture/formation_manager`

 /formation_inc_scale(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/formation_manager`

 /formation_dec_scale(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/formation_manager`

 /set_planner_positions(trajectory_planner/SetPositions)
    From :doc:`/ros_architecture/trajectory_planner`

 /plan_trajectories(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/trajectory_planner`

 /pub_trajectories(`std_srvs/Empty`_)
    From :doc:`/ros_architecture/trajectory_planner`

Parameters
^^^^^^^^^^
~n_cf(int)

~take_off_height(float)

~gnd_height(float)

~min_dist(float)

~min_goal_dist(float)

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
.. _std_srvs/SetBool: http://docs.ros.org/api/std_srvs/html/srv/SetBool.html
