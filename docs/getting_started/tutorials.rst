Tutorials
=========

.. _tuto-controller-setup:

Setup a Controller
------------------

This tutorial will show you how to find your controller layout and add it as a configuration.

1. Connect the controller to your computer

.. note:: You may need to download additional drivers. (i.e `drivers for ps4 controller <https://github.com/chrippa/ds4drv>`_ )

2. Add new controller to conf file.

.. code-block:: yaml
    # Add to ../crazyflie_charles/ros_ws/src/formation_manager/conf/swarm_conf.yaml
    controller_name:
        axes: # Axes start at 0
            x: _
            y: _
            z: _
            yaw: _

        buttons:
            '0': _
            '1': _
            '2': _

        buttons_axes:
            '-0': DL
            '0' : DR
            '1' : DU
            '-1': DD

        max_goal:
            x: 0.20
            y: 0.20
            z: 0.10
            yaw: 0.20

2. Read controller input ::

    $ rosrun joy joy_node

3. Map joystick controls

    3.1. In a new terminal, print information ::

        $ rostopic echo /joy

    3.2 Add buttons to conf file

        Find number of each button.

        .. figure:: /images/tutorials/tuto_controller_button.png

            Result of pressing X button. When can then map X to button #2




Add New Formation
-----------------

Add New Method to API
---------------------