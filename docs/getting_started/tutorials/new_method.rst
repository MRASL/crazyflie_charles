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

            # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_controller_ros.py
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

            # .../crazyflie_charles/ros_ws/src/swarm_manager/swarm_controller_ros.py``
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


    3.3 Add method to documentation

        .. code-block:: rst

            .. .../crazyflie_charles/docs/python_api.rst

            ...

            .. autosummary::
                ...
                circle_cf

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