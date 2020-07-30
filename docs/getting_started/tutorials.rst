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

In this tutorial, you will learn how to create a **new formation** and update package **documentation**.

We are going to add a formation shaped like a sinus.

1. Define formation

    .. figure:: /images/tutorials/sin_formation.png
        :scale: 50%

        Sinus formation

    * Formation center: I choose the formation center to be at the middle of the the formation X axis

    * Scale: Scale will be the length of the formation

    * Agents repartition: Agents will be equally spaced on the axis axis. Y position will be :math:`A*sin(\omega*x)`.

2. Create new formation file

    File path: ``.../crazyflie_charles/ros_ws/src/formation_manager/scripts/sin_formation.py``

    .. literalinclude:: /images/tutorials/sin_formation_template.py

3. Init method

    Two atributes will be required: `x_agents_dist` and `frequency`, to vary formation length based on scale.

    Another attribute will be added to easily change the sinus amplitude.

    ::

        def __init__(self, min_dist):
            super(SinFormation, self).__init__(min_dist)

            self.agents_x_dist = 0 # [m]
            self.frequency = 0 # [rad]
            self.amplitude = 1 # [m]

            self.compute_min_scale()

4. Verify number of agents

    Before setting a new number of agents, it's important to make sure the number is valid.
    For instance, only perfect square numbers are valid for the **square formation**.

    However, all numbers are valid for the sinus formation.

    ::

        def set_n_agents(self, n_agents):
            # All numbers are valid
            if n_agents > 0:
                self._n_agents = n_agents
                self._n_agents_landed = 0
            else:
                rospy.loginfo("Formation: Unsuported number of agents, landing %i agents"\
                    % self._n_agents_landed)

            rospy.loginfo("Formation: %i agents in formation" % self._n_agents)

            self.update_formation_scale()
            self.compute_min_scale()

5. Compute formation attributes based on scale

    First let's compute the distance between agents ::

        self.agents_x_dist = self._scale / (self._n_agents - 1) if self._n_agents > 1 else 0

    And then the formation frequency ::

        self.frequency = (2*pi)/self._scale


    ::

        def update_formation_scale(self):
            self.agents_x_dist = self._scale / (self._n_agents - 1) if self._n_agents > 1 else 0

            self.frequency = (2*pi)/self._scale

6. Find minimum scale

    The minimum scale is defined as the scale where the minimal distance between two agents is ``R_MIN``.
    For this formation, to simplify calculations, we will consider as if the formation was a simple line.

    Hence, the min_scale is when the  distance between agents is equal to ``R_MIN`` ( or ``_min_dist``)

    ::

        def compute_min_scale(self):
            if self._n_agents > 1:
                self._min_scale = self._min_dist*(self._n_agents - 1)
            else:
                self._min_scale = 0.0

7. Compute agents position from center

    We have to compute each agent position in x, y, and z from formaiton center.

    * X position ::

        x_dist = self.agents_x_dist*i - center_offset

    .. note::

        ``center_offset = self._scale/2`` is substracted from X position since the first agent is not at the center.

    * Y positions ::

        y_dist = self.amplitude*sin(self.frequency*x_dist)

    * Z positions ::

        z_dist = 0

    Completed function ::

        def compute_formation_positions(self):
            center_offset = self._scale/2 # New line

            for i in range(self._n_agents):
                if rospy.is_shutdown():
                    break

                # Initialize agent formation goal
                self._agents_goals[i] = Position()

                # Compute formation position
                x_dist = self.agents_x_dist*i - center_offset # New line
                y_dist = self.amplitude*sin(self.frequency*x_dist) # New line
                z_dist = 0 # New line

                # Compute information from center
                center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
                self._center_dist[i] = center_dist
                self._angle[i] = theta
                self._center_height[i] = center_height

            return self._agents_goals

8. Add formation to ``formation_manager_ros``

    .. code-block:: python

        # .../crazyflie_charles/ros_ws/src/formation_manager/scripts/formation_manager_ros.py``
        ...
        from sin_formation import SinFormation # New line
        ...

        class FormationManager(object):
            ...
            def __init__(self, cf_list, min_dist, start_goal):
                ...
                #: All possible formations
                self._formations = {"square": SquareFormation(self._min_dist),
                                    "v": VFormation(self._min_dist),
                                    "pyramid": PyramidFormation(self._min_dist),
                                    "circle": CircleFormation(self._min_dist),
                                    "line": LineFormation(self._min_dist),
                                    "sin": SinFormation(self._min_dist),} # New line
                ...
            ...

9. Test new formation::

        swarm = SwarmAPI()
        swarm.set_mode("formation")
        swarm.set_formation("sin")
        swarm.take_off()

.. image:: /images/tutorials/sin-formation.gif
    :height: 400px
    :width: 400px

Add New Method to API
---------------------