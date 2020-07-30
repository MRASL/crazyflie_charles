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

Add New Method to API
---------------------

In this tutorial, we will learn all the steps required to add a new method to the Python API.
We are going to add a method that moves a single crazyflie in a **small circle**.

.. note::

    For this tutorial, it's important that you have some knowledge of ROS. If not, I recommend you
    to follow these `tutorials <http://wiki.ros.org/ROS/Tutorials>`_ (begginner level) first.

To know which files to modify, it's important to understand the package
:doc:`ROS architecture </ros_architecture/general_architecture>`.

There three main layers:

* crazyflie_ros: ROS stack to control the crazyflies with the crazyradio
* swarm_manger: Package to compute swarm positions and command
* SwarmAPI: To send command to the swarm via a Python script

This architecture was used to simplify each package.

1. Create a new service

    Commands are sent to ``swarm_manager`` using a ROS service.

    1.1 Create service

        Since ``SwarmAPI`` and ``swarm_controller`` node comminucate using ros services, we first
        need to create a new service.

        The new service, will allow to specify the name of the crazyflie and return an error if the
        name is invalid.

        Create a new file named ``CircleCf.srv`` in ``.../crazyflie_charles/ros_ws/src/swarm_manager/srv/``

        .. code-block:: none

            string cf_name
            ---
            bool valid_cf

    1.2 Add service to CMakeList

        .. code-block:: none

            ...
            ## Generate services in the 'srv' folder
            add_service_files(
            FILES
            ...
            CircleCf.srv
            )
            ...

    1.3 Build catkin workspace

        ::

            $ cd .../crazyflie_charles/ros_ws
            $ catkin_make

2. Write method in ``swarm_manager`` package

    2.1 Write ``_circle_cf_srv`` method

        ::

            # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_controller.py
            ...
            from math import sin, cos, pi
            from swarm_manager.srv import CircleCf
            ...
            class SwarmController(object):
                ...
                def _circle_cf_srv(self, srv_req):
                    cf_id = srv_req.cf_name
                    valid_cf = True

                    self._state_machine.set_state("hover")

                    if cf_id in self._crazyflies:
                        # Traj parameters
                        circle_radius = 0.5 # [m]
                        circle_time = 5 # [sec]
                        n_points = int(circle_time/0.1)  # 0.1 is publish rate

                        start_pose = self._crazyflies[cf_id].pose.pose.position
                        circle_center = [start_pose.x - circle_radius, start_pose.y, start_pose.z]

                        for i in range(n_points + 1):
                            cur_angle = i*(2*pi)/n_points

                            self._crazyflies[cf_id].goals["goal"].x = circle_center[0] +\
                                                                    circle_radius*cos(cur_angle)
                            self._crazyflies[cf_id].goals["goal"].y = circle_center[1] +\
                                                                    circle_radius*sin(cur_angle)

                            self._rate.sleep()
                    else:
                        valid_cf = False

                    return {'valid_cf': valid_cf}

    2.2 Add new service to ``swarm_controller`` node

        .. code-block:: python

            # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_controller.py``
            ...
            class SwarmController(object):
            ...
                def _init_services(self):
                    # Services
                    ...
                    rospy.Service('/circle_cf', CircleCf, self._circle_cf_srv)
                    ...
            ...

3. Add method to API

    3.1 Subscribe to new service

    .. code-block:: python

            # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_api/api.py``
            ...
            from swarm_manager.srv import CircleCf
            ...
            class SwarmAPI(object):
                ...

                def _init_services(self):
                    # Subscribe to srvs
                    rospy.loginfo("API: waiting for services")
                    ...
                    self._link_service('circle_cf', CircleCf)
                    ...
                ...

    3.2 Write method

    .. code-block:: python

        # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_api/api.py``
        ...
        class SwarmAPI(object):
            ...

            def circle_cf(self, cf_id):
                """Circle specified crazyflie around a 0.5m radius

                Note:
                    This method only works in 'Automatic' mode

                Args:
                    cf_id (str): Id of crazyflie
                """
                if self.current_mode != "automatic":
                    rospy.logerr("Swarm needs to be in automatic mode")

                else:
                    srv_res = self._services["circle_cf"](cf_name=str(cf_id))
                    valid_cf = srv_res.valid_cf

                if not valid_cf:
                    rospy.logerr("%s is an invalid crazyflie name" % cf_id)
            ...

4. Test  new method

    .. code-block:: python

        swarm = SwarmAPI()

        swarm.set_mode("automatic")
        swarm.take_off()
        rospy.sleep(3)

        swarm.circle_cf("cf_0")

        rospy.spin()

    .. figure:: /images/tutorials/new-method-demo.gif
        :scale: 75%

Update Project Documentation
----------------------------

1. Go to docs folder ::

    $ cd .../crazyflie_charles/docs

2. Build html ::

    $ make html

3. Open ``.../crazyflie_charles/docs/_build/html/index.html`` in your browser

.. note::

    All documentation is written in reStructuredText.
    A good guide can be found `here <https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`_.