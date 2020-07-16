swarm_manager
=============

To manage the flight of the swarm.

Controls the position of each CF in the swarm by publishing to /cfx/goal.

Goal of each CF computed by different nodes.

.. todo:: Elaborate

.. note::
    Swarm: Groupe of all the crazyflies

    Formation: Layout of the swarm


Usage
-----


ROS Features
------------
Subscribed Topics
^^^^^^^^^^^^^^^^^
:ref:`formation-goal` (crazyflie_driver/Position)
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

 /update_swarm_params(`std_srvs/Empty`_)
    Update parameter of all swarm

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

/get_formations_list(formation_manager/GetFormationList)

/formation_inc_scale(`std_srvs/Empty`_)

/formation_dec_scale(`std_srvs/Empty`_)

/set_planner_positions(trajectory_planner/SetPositions)

/plan_trajectories(`std_srvs/Empty`_)

/pub_trajectories(`std_srvs/Empty`_)

Parameters
^^^^^^^^^^
~cf_list(str, default: ['cf1'])
~to_sim(bool, default: False)
~take_off_height(float)
~gnd_height(float)
~min_dist(float)
~min_goal_dist(float)

.. _std_srvs/Empty: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
.. _std_srvs/SetBool: http://docs.ros.org/api/std_srvs/html/srv/SetBool.html
