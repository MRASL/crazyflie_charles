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

Architecture
------------
To control the swarm, three different ros packages are used:

* :doc:`/architecture/swarm_manager`: Main package. Link between the other packages and `crazyflie ros stack`_.
* :doc:`/architecture/formation_manager`: Handles movement of swarm in a specific formation
* :doc:`/architecture/trajectory_planner`: Moves agents between positions. Used to change formation

The general architecture can be found here :doc:`/architecture/general_architecture`.

.. toctree::
   :maxdepth: 2
   :hidden:
   :caption: Architecture

   architecture/general_architecture
   architecture/swarm_manager
   architecture/formation_manager
   architecture/trajectory_planner

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

Indices and tables
------------------
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


.. _crazyflie ros stack: https://github.com/whoenig/crazyflie_ros