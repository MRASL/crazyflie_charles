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

        self.frequency = (2*pi)/self._scale if self._scale > 0 else 0


    ::

        def update_formation_scale(self):
            self.agents_x_dist = self._scale / (self._n_agents - 1) if self._n_agents > 1 else 0

            self.frequency = (2*pi)/self._scale if self._scale > 0 else 0

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


.. note:: Completed sinus formation file can be found in ``.../crazyflie_charles/demos``