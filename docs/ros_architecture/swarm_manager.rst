Swarm Manager
=============

Three nodes are used in this package:

* :doc:`/ros_architecture/swarm_manager/swarm_manager`: Main node. Controls commands sent to all CFs.
* :doc:`/ros_architecture/swarm_manager/joy_controller`: Send joystick data to swarm_manager.
* :doc:`/ros_architecture/swarm_manager/cfx_controller`: Controls a single crazyflie to match swarm_manager output

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Nodes

   /ros_architecture/swarm_manager/swarm_manager
   /ros_architecture/swarm_manager/joy_controller
   /ros_architecture/swarm_manager/cfx_controller