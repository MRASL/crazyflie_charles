#!/usr/bin/env python

"""
Python script to analyse flight data

.. warning::
    Data are not in perfect sync. Their can be a small delay between the two ~0.1 secf

Options:
    - [x] Specify data name
    - [x] Plot position : exp pose and goal
        - [x] Plot position
        - [x] Specify axis
        - [x] Draw a line
        - [x] Draw goal
        - [x] Possibility to accelerate/slow
        - [x] 3D plot
        - [x] Seperate 3D and 2D class
    - [x] List CF names
    - [x] Save as
    - [ ] Data analysis
    - [x] UI
        - [x] Call desired function in terminal
        - [x] Assert data types
        - [x] Verify args and optional args
        - [x] Print help
"""

import os
from os.path import isfile, join
import sys
import argparse
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.axes3d as p3

from user_command import UserInterface, InputError

class DataAnalyser(object):
    """
    To analyse data
    """
    def __init__(self, data_name, to_plot):
        self.data_name = data_name
        self.to_plot = to_plot
        self.data_path = None
        self.data_base_name = 'flight_data_'
        self.crazyflies = None

        self.load_data(self.data_name)

        self.ui_cmd = UserInterface()
        self._add_commands()

    def _add_commands(self):
        # Load data
        self.ui_cmd.add_cmd('load', self.load_data, "Load a trajectory")
        self.ui_cmd.add_arg('load', 'data_name', "Name of data file",
                            arg_type=str, optional=False)

        # Exit
        self.ui_cmd.add_cmd('exit', self.exit, "Exit program")

        # Plot data
        self.ui_cmd.add_cmd('plot_data', self.plot_data, "To plot a CF trajectory")
        self.ui_cmd.add_arg('plot_data', 'cf_id', "Id of crazyflie",
                            arg_type=str, optional=False)
        self.ui_cmd.add_arg('plot_data', 'plot2d', "To plot in 2d",
                            arg_type=bool, optional=True)
        self.ui_cmd.add_arg('plot_data', 'axes', "Specify axes of 2d plot. i.e: 'xz'",
                            arg_type=str, optional=True)

        # List CFs
        self.ui_cmd.add_cmd('list_cf', self.list_cf, "List all CFs")

        # Rename file
        self.ui_cmd.add_cmd('rename', self.rename_data, "Rename a file")
        self.ui_cmd.add_arg('rename', 'new_name', "Name of data file",
                            arg_type=str, optional=False)

    def _find_latest_data(self):
        """Find latest data name

        Returns:
            str: Name of latest data set
        """
        # List all data files
        data_list = [f for f in os.listdir(self.data_path) if isfile(join(self.data_path, f))]
        latest_id = None

        # Find all unnamed data
        if data_list:
            data_list = [f for f in data_list if f.startswith(self.data_base_name)]

            if data_list:
                data_list = [f.replace(self.data_base_name, '') for f in data_list]
                data_list = [int(f.replace('.npy', '')) for f in data_list]
                latest_id = str(np.max(data_list))

        if latest_id is not None:
            latest_data = self.data_base_name + latest_id + '.npy'
        else:
            raise RuntimeError("Data not found")

        return latest_data

    # User commands
    def prompt_user(self):
        """Wait for user command
        """

        user_cmd = raw_input("\nEnter Command: ")

        try:
            func, args, kwargs = self.ui_cmd.get_command(user_cmd)

            if func is not None:
                func(*args, **kwargs)

        except InputError as err:
            print "\tInput Error: %s" % err.message

    def load_data(self, data_name):
        """Load flight data.

        If data_name is empty, load latest data set
        """
        parentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.data_path = os.path.join(parentdir, 'flight_data')

        if data_name == "":
            self.data_name = self._find_latest_data()

        else:
            self.data_name = data_name + '.npy'

        data_path = join(self.data_path, self.data_name)

        try:
            self.crazyflies = np.load(data_path, allow_pickle='TRUE').item()
        except IOError:
            print "\tFile not found: %s" % self.data_name

        print "\tFile loaded: %s" % self.data_name

    def rename_data(self, new_name):
        """Rename data file
        """
        # Save data
        file_path = join(self.data_path, new_name)
        np.save(file_path, self.crazyflies)

        # Delete old file
        old_file_path = join(self.data_path, self.data_name)
        os.remove(old_file_path)

        self.data_name = new_name + '.npy'
        print "\t New data name: %s" % self.data_name

    def exit(self):
        """Exit program
        """
        sys.exit(0)

    def list_cf(self):
        """To list name of all CFs
        """
        print "Crazyflie name in data:"
        for cf_id in sorted(self.crazyflies.keys()):
            print "\t%s" % cf_id

    def plot_data(self, cf_id, axes='xy', plot2d=False):
        """Plot flight of specified crazyflie.

        Plot axis can be specified

        Args:
            cf_id (str): Name of crazyflie
            axis (str, optional): Axes to plot data. Defaults to 'xy'.
            plot2d (bool, optiona;): To plot in 2d or not. Defaults to False.
        """
        flight_data = self.crazyflies[cf_id]

        if not plot2d:
            plotter = DataPlotter3d(flight_data)
        else:
            plotter = DataPlotter2d(axes, flight_data)

        plotter.plot_traj()



class DataPlotter3d(object):
    """To plot flight data
    """
    def __init__(self, flight_data, anim_rate=1.0):
        self.flight_data = flight_data
        self._sync_data()

        self.data_freq = 10.0 # Hz

        self.anim_rate = anim_rate  #: int: To accelerate animation

        self.fig = plt.figure()

        self.axes = p3.Axes3D(self.fig)
        self.axes.set_xlabel('x (m)')
        self.axes.set_xlim(0, 5)

        self.axes.set_ylabel('y (m)')
        self.axes.set_ylim(0, 5)

        self.axes.set_zlabel('z (m)')
        self.axes.set_zlim(0, 2.5)

        self.axes.set_title('Trajectories')

        self.axes.set_aspect('equal', adjustable='box')

        self.animated_objects = [] # List of all objects to animate
        self.time_text = None

        self._init_animated_objects()

        self.cf_data = {'pose':[[], []], 'goal': [[], []]} #: Each data, [[axis_0], [axis_1]]

    def __del__(self):
        plt.close()

    def _sync_data(self):
        # Sync data so they start at the sime time
        # Remove data in extra (received at the beginning)
        extra_data = len(self.flight_data['pose']) - len(self.flight_data['goal'])

        if extra_data > 0:
            self.flight_data['pose'] = self.flight_data['pose'][extra_data::]

        else:
            self.flight_data['goal'] = self.flight_data['goal'][-extra_data::]

    def _init_animated_objects(self):
        """Creates all objects to animate.
        """
        # CF pose
        line = self.axes.plot([], [], [], lw=2, color='b', marker='o')[0]
        self.animated_objects.append(line)

        line = self.axes.plot([], [], [], lw=2, color='b')[0]
        self.animated_objects.append(line)

        # CF goal
        line = self.axes.plot([], [], [], lw=1, alpha=0.8, color='r', marker='o')[0]
        self.animated_objects.append(line)

        line = self.axes.plot([], [], [], lw=1, color='r')[0]
        self.animated_objects.append(line)

        # Add time_text
        self.time_text = self.axes.text2D(0.02, 0.95, '', transform=self.axes.transAxes)
        self.animated_objects.append(self.time_text)

    def _init_animation(self):
        # Pose circle
        start_pose = self.flight_data['pose'][0].pose.position

        self.animated_objects[0].set_data(start_pose.x, start_pose.y)
        self.animated_objects[0].set_3d_properties(start_pose.z)

        self.animated_objects[1].set_data([], [])
        self.animated_objects[1].set_3d_properties([])

        # Goal circle
        start_goal = self.flight_data['goal'][0]

        self.animated_objects[2].set_data(start_goal.x, start_goal.y)
        self.animated_objects[2].set_3d_properties(start_goal.z)

        self.animated_objects[3].set_data([], [])
        self.animated_objects[3].set_3d_properties([])


        # Set text
        self.animated_objects[-1].set_text('')

        return self.animated_objects

    def _animate(self, frame):
        """Animate

        Args:
            frame (int): Current frame
        """
        if frame == 0:
            self.cf_data = {'pose':[[], [], []], 'goal': [[], [], []]}

        # CF pose
        cf_pose = self.flight_data['pose'][frame].pose.position
        self.cf_data['pose'][0].append(cf_pose.x)
        self.cf_data['pose'][1].append(cf_pose.y)
        self.cf_data['pose'][2].append(cf_pose.z)


        self.animated_objects[0].set_data(cf_pose.x, cf_pose.y)
        self.animated_objects[0].set_3d_properties(cf_pose.z)

        self.animated_objects[1].set_data(self.cf_data['pose'][0:2])
        self.animated_objects[1].set_3d_properties(self.cf_data['pose'][2])


        # CF goal
        cf_goal = self.flight_data['goal'][frame]
        self.cf_data['goal'][0].append(cf_goal.x)
        self.cf_data['goal'][1].append(cf_goal.y)
        self.cf_data['goal'][2].append(cf_goal.z)


        self.animated_objects[2].set_data(cf_goal.x, cf_goal.y)
        self.animated_objects[2].set_3d_properties(cf_goal.z)

        self.animated_objects[3].set_data(self.cf_data['goal'][0:2])
        self.animated_objects[3].set_3d_properties(self.cf_data['goal'][2])

        time = frame/self.data_freq
        self.time_text.set_text("Time (sec): %.1f" % time)

        # print "Pose time: sec: %i \t nsec: %i\t Goal time: sec: %i \t nsec: %i" %\
        #     (self.flight_data['pose'][frame].header.stamp.secs,
        #      self.flight_data['pose'][frame].header.stamp.nsecs,
        #      self.flight_data['goal'][frame].header.stamp.secs,
        #      self.flight_data['goal'][frame].header.stamp.nsecs,)

        return self.animated_objects

    def plot_traj(self):
        """To run animation
        """
        self._init_animation()

        n_frame = np.min([len(self.flight_data['pose']), len(self.flight_data['goal'])])

        _ = FuncAnimation(self.fig, self._animate, init_func=self._init_animation,
                          frames=n_frame, interval=(1000/(self.data_freq*self.anim_rate)),
                          blit=False)

        plt.show()

class DataPlotter2d(object):
    """To plot flight data
    """
    def __init__(self, axes, flight_data, anim_rate=1.0):
        self.flight_data = flight_data
        self._sync_data()

        self.axes_name = axes
        self.axes_id = None
        self._find_axes_id()

        self.data_freq = 10.0 # Hz

        #: int: Number of frame, corresponds to number of column
        self.n_frame = np.min([len(self.flight_data['pose']), len(self.flight_data['goal'])])
        self.anim_rate = anim_rate  #: int: To accelerate animation

        self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.axes = plt.axes(xlim=(-1, 5), ylim=(-1, 5))
        self.axes.set_title('Trajectories')
        self.axes.set_xlabel(self.axes_name[0] + ' (m)')
        self.axes.set_ylabel(self.axes_name[1] + ' (m)')
        self.axes.set_aspect('equal', adjustable='box')

        self.animated_objects = [] # List of all objects to animate
        self.time_text = None

        self._init_animated_objects()

        self.cf_data = {'pose':[[], []], 'goal': [[], []]} #: Each data, [[axis_0], [axis_1]]

    def __del__(self):
        plt.close()

    def _find_axes_id(self):
        # Find axes ids
        axes = self.axes_name.replace('x', '0')
        axes = axes.replace('y', '1')
        axes = axes.replace('z', '2')
        self.axes_id = [int(axes[0]), int(axes[1])]

    def _sync_data(self):
        # Sync data so they start at the sime time
        # Remove data in extra (received at the beginning)
        extra_data = len(self.flight_data['pose']) - len(self.flight_data['goal'])

        if extra_data > 0:
            self.flight_data['pose'] = self.flight_data['pose'][extra_data::]

        else:
            self.flight_data['goal'] = self.flight_data['goal'][-extra_data::]

    def _init_animated_objects(self):
        """Creates all objects to animate.
        """
        # CF pose
        circle = Circle((0, 0), 0.1, alpha=0.8, fc='b')
        self.axes.add_patch(circle)
        self.animated_objects.append(circle)

        line, = self.axes.plot([], [], lw=2, color='b')
        self.animated_objects.append(line)

        # CF goal
        circle = Circle((0, 0), 0.05, alpha=0.8, fc='r')
        self.axes.add_patch(circle)
        self.animated_objects.append(circle)

        line, = self.axes.plot([], [], lw=1, color='r')
        self.animated_objects.append(line)

        # Add time_text
        self.time_text = self.axes.text(0.02, 0.95, '', transform=self.axes.transAxes)
        self.animated_objects.append(self.time_text)

    def _init_animation(self):
        # Pose circle
        start_pose = self.flight_data['pose'][0].pose.position
        start_pose = [start_pose.x, start_pose.y, start_pose.z]

        self.animated_objects[0].center = (start_pose[self.axes_id[0]], start_pose[self.axes_id[1]])
        self.animated_objects[1].set_data([], [])

        # Goal circle
        start_goal = self.flight_data['goal'][0]
        start_goal = [start_goal.x, start_goal.y, start_goal.z]

        self.animated_objects[2].center = (start_goal[self.axes_id[0]], start_goal[self.axes_id[1]])
        self.animated_objects[3].set_data([], [])

        # Set text
        self.animated_objects[-1].set_text('')

        return self.animated_objects

    def animate(self, frame):
        """Animate

        Args:
            frame (int): Current frame
        """
        if frame == 0:
            self.cf_data = {'pose':[[], []], 'goal': [[], []]}

        # CF pose
        cf_pose = self.flight_data['pose'][frame].pose.position
        cf_pose = [cf_pose.x, cf_pose.y, cf_pose.z]
        self.cf_data['pose'][0].append(cf_pose[self.axes_id[0]])
        self.cf_data['pose'][1].append(cf_pose[self.axes_id[1]])

        self.animated_objects[0].center = (cf_pose[self.axes_id[0]], cf_pose[self.axes_id[1]])
        self.animated_objects[1].set_data(self.cf_data['pose'])

        # CF goal
        cf_goal = self.flight_data['goal'][frame]
        cf_goal = [cf_goal.x, cf_goal.y, cf_goal.z]
        self.cf_data['goal'][0].append(cf_goal[self.axes_id[0]])
        self.cf_data['goal'][1].append(cf_goal[self.axes_id[1]])

        self.animated_objects[2].center = (cf_goal[self.axes_id[0]], cf_goal[self.axes_id[1]])
        self.animated_objects[3].set_data(self.cf_data['goal'])

        time = frame/self.data_freq
        self.time_text.set_text("Time (sec): %.1f" % time)

        # print "Pose time: sec: %i \t nsec: %i\t Goal time: sec: %i \t nsec: %i" %\
        #     (self.flight_data['pose'][frame].header.stamp.secs,
        #      self.flight_data['pose'][frame].header.stamp.nsecs,
        #      self.flight_data['goal'][frame].header.stamp.secs,
        #      self.flight_data['goal'][frame].header.stamp.nsecs,)

        return self.animated_objects

    def plot_traj(self):
        """To run animation
        """
        self._init_animation()

        _ = FuncAnimation(self.fig, self.animate, init_func=self._init_animation,
                          frames=self.n_frame, interval=(1000/(self.data_freq*self.anim_rate)),
                          blit=True)

        plt.show()

if __name__ == '__main__':
    # pylint: disable=invalid-name
    parser = argparse.ArgumentParser(description='Analyse flight data')
    parser.add_argument('--data-name', '-d', type=str, help='Name of data', default='')
    parser.add_argument('--plot', '-p', action='store_true', help='To plot data', default=False)

    parsed_args = parser.parse_args()
    d_name = parsed_args.data_name
    plot = parsed_args.plot

    analyser = DataAnalyser(d_name, plot)
    # pylint: enable=invalid-name

    while True:
        analyser.prompt_user()
