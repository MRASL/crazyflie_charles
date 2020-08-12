Swarm Manager
=============

Overview of package architecture:

.. image:: /images/swarm_manager_architecture.svg


For an in depth description of each ros node:

* :doc:`/ros_architecture/swarm_manager/swarm_api`: Python API
* :doc:`/ros_architecture/swarm_manager/swarm_controller_ros`: Main node. Controls commands sent to all CFs.
* :doc:`/ros_architecture/swarm_manager/joy_controller`: Send joystick data to swarm_manager.
* :doc:`/ros_architecture/swarm_manager/cf_controller`: Controls a single crazyflie to match swarm_manager output.
* :doc:`/ros_architecture/swarm_manager/cf_sim`: Simulate position of a crazyflie.
* :doc:`/ros_architecture/swarm_manager/cf_broadcaster`: Broadcast position of a crazyflie to view in RVIZ
* :doc:`/ros_architecture/swarm_manager/flight_recorder`: To record and save all CFs trajectories.

For a description of all ros topics used, see :doc:`/ros_architecture/topics`.

.. toctree::
   :maxdepth: 1
   :hidden:
   :caption: Nodes

   /ros_architecture/swarm_manager/swarm_api
   /ros_architecture/swarm_manager/swarm_controller_ros
   /ros_architecture/swarm_manager/joy_controller
   /ros_architecture/swarm_manager/cf_controller
   /ros_architecture/swarm_manager/cf_sim
   /ros_architecture/swarm_manager/cf_broadcaster
   /ros_architecture/swarm_manager/flight_recorder