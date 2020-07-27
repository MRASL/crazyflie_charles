.. Crazyflie Swarm Controller documentation master file, created by
   sphinx-quickstart on Thu Jun 25 09:06:26 2020.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Crazyflie Swarm Controller documentation
========================================

.. todo:: Ajouter description et exemple
.. todo:: Ajouter features, sim, trajectory, formation...

First Steps
-----------

installation, dependencies, ressources

.. todo:: Required skills, suggested reading

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: First steps

   first_steps/installation
   first_steps/ressources

Ros Architecture
----------------
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

User Guide
----------
.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: User Guide

Package Description
-------------------
In-depth description of each package

.. toctree::
   :maxdepth: 2
   :caption: Package Description

   package_description/swarm_manager
   package_description/formation_manager
   package_description/trajectory_planner

Glossary, Indices and tables
----------------------------

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Glossary

   glossary

* :doc:`glossary`
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`



.. _crazyflie ros stack: https://github.com/whoenig/crazyflie_ros

.. [CIT1] C. E. Luis and A. P. Schoellig,
   "Trajectory Generation for Multiagent Point-To-Point Transitions via Distributed Model Predictive Control,"
   in IEEE Robotics and Automation Letters, vol. 4, no. 2, pp. 375-382, April 2019, doi: 10.1109/LRA.2018.2890572.
