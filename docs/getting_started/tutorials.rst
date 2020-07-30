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

        .. code-block:: yaml

            controller_name:
                ...

                buttons:
                    '0': _
                    '1': _
                    '2': X

                ...

    3.3 Add axes to conf file

        You have four control to map: *x position*, *y position*, *z position* and *yaw*.

        Let's say you want to map the horizontal right stick to yaw control. You get this result when
        moving the stick to the left.

        .. figure:: /images/tutorials/tuto_controller_axis.png

            Result of moving the right stick to the left.

        With this we determine that the horizontal right stick axis number is 2. Also, since moving
        it to the left gives a positive result, the axis number will be negative:

        .. code-block:: yaml

            controller_name:
                ...

                axes:
                    x:
                    y:
                    z:
                    yaw: -2


                ...

        Repeat this for all controls you wish to map.

4. Add controller to API::

    # .../crazyflie_charles/ros_ws/src/swarm_manager/scripts/swarm_api/api.py
    class SwarmAPI(object):
    ...
    def start_joystick(self, joy_type=""):
    """Initialize joystick node

        Possible types are:
            - ds4
            - ADD NEW CONTROLLER

        ...
    """
    ...

5. Try new controller with api by using ::

    SwarmAPI.start_joystick("new_controller")


Add New Formation
-----------------



Add New Method to API
---------------------