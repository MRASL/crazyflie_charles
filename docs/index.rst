.. Crazyflie Swarm Controller documentation master file, created by
   sphinx-quickstart on Thu Jun 25 09:06:26 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

****************************************
Crazyflie Swarm Controller documentation
****************************************

Overview
========

`Crazyflie Swarm Controller` is a ROS package to fly a swarm of crazyflie in formation.

Main features:

* Python API
* Collision free trajectory planning via a DMPC algorithm
* Fly swarm in various formations (square, circle, line...)
* Simulation
* Easy control with a joystick

This project was developped with the **Mobile Robotics and Autonomous Systems Laboratory** at
**Polytechnique Montréal**.

To cite this project:

.. todo:: Add citation

Exemple
-------

Formation Exemple
^^^^^^^^^^^^^^^^^
   ::

      # Formation exemple
      swarm = SwarmAPI()

      # Link joystick buttons to commands
      swarm.start_joystick("ds4")
      swarm.link_joy_button("S", swarm.take_off)
      swarm.link_joy_button("X", swarm.land)
      swarm.link_joy_button("O", swarm.emergency)
      swarm.link_joy_button("T", swarm.toggle_ctrl_mode)

      # Start swarm
      swarm.set_mode("formation")
      swarm.set_formation("v")

      swarm.take_off()
      rospy.sleep(10)

      # Change formation
      swarm.set_formation("pyramid")


High level controller exemple
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   ::

      # Trade spots demo
      swarm = SwarmAPI()
      swarm.set_mode("automatic")

      # Take off
      swarm.take_off()

      # Switch positions
      pose = swarm.get_positions()
      goals = {}
      goals["cf_0"] = pose["cf_1"]
      goals["cf_1"] = pose["cf_0"]
      swarm.go_to(goals)

      rospy.sleep(10)

      # Land
      swarm.land()


.. |video1| image:: /images/formation-demo.gif
   :height: 400px
   :width: 400px

.. |video2| image:: /images/cf-spots.gif
   :height: 400px
   :width: 400px

.. table::
   :align: center

   +----------+----------+
   | |video1| | |video2| |
   +----------+----------+


.. Getting started

.. toctree::
   :maxdepth: 2
   :caption: Getting started

   getting_started/installation
   getting_started/usage
   getting_started/tutorials
   getting_started/mrasl_demo

Python API
==========

:doc:`/python_api`

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Python API

   python_api

Files Tree
==========

.. code-block:: none

   |-- README.md
   |-- build.sh: Project build script
   |-- pc_persmissions.sh
   |-- requirements.txt
   |-- docs: Folder with all thing related to documentation
   |-- demos: Example scripts
   |
   |-- flight_data
       |-- flight_analysis.py: Script to analyse flight flight data
       |-- user_command.py: Script to read user input
       |-- all flight data...
   |
   |-- ros_ws
       |-- build
       |-- devel
       |-- src
           |-- CMakeLists.txt
           |-- crazyflie_ros
           |-- formation_manager
           |-- swarm_manager
           |-- trajectory_planner

ROS Architecture
================
To control the swarm, three different ros packages are used:

* :doc:`/ros_architecture/swarm_manager`: Main package. Link between the other packages and `crazyflie ros stack`_. Includes a python api.
* :doc:`/ros_architecture/formation_manager`: To move the swarm in a specific formation (i.e square, circle, ...)
* :doc:`/ros_architecture/trajectory_planner`: To move agents without collisions. Used to change formation

The general architecture can be found here :doc:`/ros_architecture/general_architecture`.

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Ros Architecture

   ros_architecture/general_architecture
   ros_architecture/swarm_manager
   ros_architecture/formation_manager
   ros_architecture/trajectory_planner
   ros_architecture/topics

Package Description
===================
In-depth description of each package

.. toctree::
   :maxdepth: 1
   :caption: Package Description

   package_description/swarm_manager
   package_description/formation_manager
   package_description/trajectory_planner

References
==========

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: References

   glossary
   ressources
   bibliography

* :doc:`glossary`
* :doc:`ressources`
* :doc:`bibliography`
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`



.. _crazyflie ros stack: https://github.com/whoenig/crazyflie_ros
