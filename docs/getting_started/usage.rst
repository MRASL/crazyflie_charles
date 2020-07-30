Usage
=====

Crazyflie Assembly and Test
---------------------------

Before going further, follow this `tutorial <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/>`_.
It explains how to assemble a crazyflie and install the `PC client for linux <https://github.com/bitcraze/crazyflie-clients-python/blob/master/README.md>`_.


Swarm Setup
-----------

1.  Set crazyflie address. Each crazyflie of the swarm needs to have a unique address.
    The address format is ::

        0xE7E7E7E7<X>

    where ``<X>`` is the crazyflie number in hexadecimal.

.. note:: It's highly recommended to label each CF with it's number.

2.  Set crazyflie channel. Each crazyradio can handle communication with up to ___ crazyflies.
    If more crazyflies are in your swarm, you will need a different channel for each radio.

3.  If not already done,
    `update each crazyflie firmware <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#config-client>`_


Positionning System
-------------------

For now, only the LPS (with 8 anchors) was tested for positionning. Follow `this guide <https://www.bitcraze.io/documentation/tutorials/getting-started-with-loco-positioning-system/>`_
to setup your system.

It's possible to test your setup by trying to fly with it: `here <https://www.bitcraze.io/documentation/tutorials/getting-started-with-flying-using-lps/>`_


Configuration Files
-------------------

There are two configuration files:

    * ``swarm_conf.yaml``
    * ``joy_conf.yaml``

They are located at: ``../crazyflie_charles/ros_ws/src/formation_manager/conf``

swarm_conf
^^^^^^^^^^

This file is used to configure parameters of all three ros packages. It's where you can change
the number of crazyflie in the swarm.

.. code-block:: yaml

    # ../crazyflie_charles/ros_ws/src/formation_manager/conf/swarm_conf.yaml
    swarm:
        n_cf: 2 # Number of CF in the swarm
        take_off_height: 0.5
        gnd_height: 0.0
        min_dist: 0.40  # Absolute min distance between CFs
        min_goal_dist: 0.40 # Absolute min distance between CFs goals

    formation:
        formation_min_dist: 0.6 # Min distance between agents in formation
        formation_start_pos: [0.5, 0.5, 0.5, 0.0] # [x, y, z, yaw]

    trajectory_solver:
        # trajectory_solver parameters ...
        ...

    # Starting positions. Used in simulation
    starting_positions:
        cf_0: [0.0, 0.0, 0.0]
        cf_1: [0.0, 0.5, 0.0]
        cf_2: [0.0, 1.0, 0.0]
        cf_3: [0.5, 0.0, 0.0]
        ...

joy_conf
^^^^^^^^

File used to map your controller buttons. To learn how to setup a new a new controller, see this
:ref:`tutorial <tuto-controller-setup>`.

.. note:: If you are using a ps4 controller (dualshock), you will need to download `this driver <https://github.com/chrippa/ds4drv>`_.

.. code-block:: yaml

    # ../crazyflie_charles/ros_ws/src/formation_manager/conf/swarm_conf.yaml
    ds4:
        # Map axes and joystick stick number
        axes: # Axes start at 0
            x: 1
            y: 0
            z: 5
            yaw: 2

        # Map button name and position
        buttons:
            '0': S # Square
            '1': X # Cross
            '2': O # Circle
            '3': T # Triangle
            ...

        # Map buttons on a joystick axis, i.e: d-pad
        buttons_axes:
            '9': DL
            '-9' : DR
            '10' : DU
            '-10': DD

        # Max velocity of goal
        max_goal:
            x: 0.20
            y: 0.20
            z: 0.10
            yaw: 0.20

Flying
------

1. Turn on and place all your CFs in the flight alrea

2.  Launch ros server ::

    $ roslaunch swarm_manager launch_swarm.launch

    There are two options when launching server:

    * ``sim:=bool`` (default: True): To run in simulation
    * ``save:=bool`` (default: False): To save flight data when closing server

3. In another terminal, execute python script ::

    $ cd ../crazyflie_charles/demos
    $ python trade_spots.py

Data Analysis
-------------

A python script allow to analyse the data took. To run the script ::

    $ cd ../crazyflie_charles/flight_data
    $ python flight_analysis.py

.. note:: It's possible to specify a file name using -d flag.
          If no file name specified, latest data will be loaded.

Possible commands:

    * Rename data set
    * List all cf in recorded data
    * Plot flight path of a crazyflie
    * Plot trajectory error

.. note:: Enter ``help`` to print all commands and their arguments.