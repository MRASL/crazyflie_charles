Python API
==========

.. currentmodule:: swarm_api.api.SwarmAPI

.. autosummary::

    start_joystick
    link_joy_button
    take_off
    stop
    emergency
    land
    set_mode
    set_formation
    next_formation
    prev_formation
    inc_scale
    dec_scale
    toggle_ctrl_mode
    go_to
    get_positions
    rotate_formation

Python API to control the swarm.

This module maked it possible to easily send command to the swarm through a Python script.

Example
-------
::

   # Formation exemple
   swarm = SwarmAPI()

   # Link joystick buttons to commands
   swarm.start_joystick("ds4")
   swarm.link_joy_button("S", swarm.take_off)
   swarm.link_joy_button("X", swarm.land)
   swarm.link_joy_button("O", swarm.emergency)
   swarm.link_joy_button("T", swarm.toggle_ctrl_mode)

   # Start swarm
   swarm.set_mode("formation")
   swarm.set_formation("v")

   swarm.take_off()
   rospy.sleep(10)

   # Change formation
   swarm.set_formation("pyramid")

``SwarmAPI`` class
------------------

.. autoclass:: swarm_api.api.SwarmAPI
    :members: