Installation
============

OS Requirement
--------------

This package was made and tested for **Ubuntu 18.04 with ROS Melodic**.

The following links might be useful:

* `Dual boot windows 10 and linux <https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/>`_
* `Ubuntu 18.04 <https://releases.ubuntu.com/18.04/>`_
* `ROS Melodic installation <http://wiki.ros.org/melodic/Installation/Ubuntu>`_

.. note:: It might be possible to use this package with a VM. Please let me know if you try it and it works.
    Note that a VM will add latency.

Package installation
--------------------

1 - Verify Python version. Only **Python 2.7.17** was tested. ::

    $ python --version

2 - Clone package. ::

    $ git clone https://github.com/MRASL/crazyflie_charles.git

3 - Install python dependencies and build package using build script. ::

    $ cd crazyflie_charles
    $ ./build.sh

4 - Modify pythonpath so venv can run ::

    $ echo "export PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages" >> ~/.bashrc
    $ source ~/.bashrc

.. warning:: When ``/usr/lib/python2.7/dist-packages`` is in ``PYTHONPATH``, you won't be able
             to download packages through ``pip``

5 - Test installation. ::

    $ source ros_ws/devel/setup.sh
    $ roslaunch swarm_manager launch_swarm.launch
    $ python demos/trade_spots.python

.. warning:: Make sure your ros environment has been source and roscore is running
    before testing this exemple. `See section 1.5 <http://wiki.ros.org/melodic/Installation/Ubuntu>`_.

6 - (optional) Automaticaly source ros workspace by adding it to .bashrc ::

    $ echo "source <path_to_crazyflie_charles>/ros_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc

.. note::   | Replace ``<path_to_crazyflie_charles>`` with your installation path.
            | i.e: ``~/projects/crazyflie_charles``
