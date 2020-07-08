#!/usr/bin/env python
"""
Trajectory Plotter
------------------

Module to plot the trajectories of agents.

Circles represent the agents, dashed line the predicted trajectory over the horizon

"""

# import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

plt.style.use('seaborn-pastel')

SAVE_ANIMATION = False
N_PER_AGENT = 3

class TrajPlot(object):
    """To plot trajectories of agents
    """
    # pylint: disable=too-many-instance-attributes
    # 11 is reasonable in this case.

    def __init__(self, agent_list, time_step, wait_for_input=False):
        """Init

        Args:
            x (array): Trajectories
            h (float): Time step
        """
        self.agents = agent_list # Position and acceleration at each time step
        self.n_agents = len(self.agents)
        self.time_step = time_step # Time step
        self.wait_for_input = wait_for_input

        #: int: Number of frame, corresponds to number of column
        self.n_frame = self.agents[-1].states.shape[1]
        self.slow_rate = 1  #: int: To slow animation

        self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.axes = plt.axes(xlim=(-1, 5), ylim=(-1, 5))
        self.axes.set_title('Trajectories')
        self.axes.set_xlabel('x (m)')
        self.axes.set_ylabel('y (m)')
        self.axes.set_aspect('equal', adjustable='box')

        self.color_list = ['b', 'r', 'g', 'c', 'm', 'y', 'k']
        self.animated_objects = [] # List of all objects to animate
        self.init_animated_objects()

    def __del__(self):
        plt.close()

    # Setters
    def set_wait_for_input(self, to_wait):
        """To wait or not for input before switching frame

        Args:
            to_wait (bool): To wait
        """
        self.wait_for_input = to_wait

    def set_slow_rate(self, slow_rate):
        """Set slow rate of animation.

        Rate of 1 is real time. Rate of 2 is twice slower

        Args:
            slow_rate (float): Rate of slow
        """
        self.slow_rate = slow_rate

    def set_axes_limits(self, xlim, ylim):
        """Set x and y axes limits

        Args:
            xlim (tuple): (xmin, xmax)
            ylim (tuple): (ymin, ymax)
        """
        self.axes.set_xlim(xlim)
        self.axes.set_ylim(ylim)

    # Animation
    def init_animated_objects(self):
        """Creates all objects to animate.

        Each agent has:
            - A circle (current position)
            - A dashed line (predicted trajectory)
            - An X (goal)

        Notes:
            Structure of animated object. Idx:
                0: circle of agent 1
                1: line of agent 1
                2: circle of agent 2
                3: line of agent 2
                ...
                -1: time text

        """
        color_idx = 0
        for each_agent in self.agents:
            color = self.color_list[color_idx%len(self.color_list)]
            circle = Circle((0, 0), 0.1, alpha=0.8, fc=color)
            line, = self.axes.plot([], [], lw=2, linestyle='dashed', color=color)#, marker='o')

            col_circle = Circle((0, 0), 0.45, alpha=0.2, fc=color)

            self.axes.add_patch(circle)
            self.axes.add_patch(col_circle)

            self.animated_objects.append(circle)
            self.animated_objects.append(col_circle)
            self.animated_objects.append(line)

            # Draw goal
            x_goal = each_agent.goal[0]
            y_goal = each_agent.goal[1]
            self.axes.scatter(x_goal, y_goal, s=250, c=color, marker='X')

            color_idx += 1

        # Add time_text
        self.time_text = self.axes.text(0.02, 0.95, '', transform=self.axes.transAxes)
        self.animated_objects.append(self.time_text)

    def init_animation(self):
        """Initialize animation
        """
        for i in range(self.n_agents):
            agent = self.agents[i]

            # Circle
            self.animated_objects[N_PER_AGENT*i].center = (agent.states[0, 0],
                                                           agent.states[1, 0])

            # Col Circle
            self.animated_objects[N_PER_AGENT*i].center = (agent.states[0, 0],
                                                           agent.states[1, 0])

            # Line
            self.animated_objects[N_PER_AGENT*i+2].set_data([], [])

        # Set text
        self.animated_objects[-1].set_text('')

        return self.animated_objects

    def animate(self, frame):
        """Animate

        Args:
            frame (int): Current frame
        """

        for i in range(self.n_agents):
            agent = self.agents[i]
            data = agent.states[:, frame]

            # Circle
            self.animated_objects[N_PER_AGENT*i].center = (data[0], data[1])
            self.animated_objects[N_PER_AGENT*i + 1].center = (data[0], data[1])

            # Prediction line
            x_data = []
            y_data = []
            z_data = []

            for k in range(int(len(data)/6)):
                x_data.append(data[6*k])
                y_data.append(data[6*k + 1])
                z_data.append(data[6*k + 2])

            self.animated_objects[N_PER_AGENT*i + 2].set_data(x_data, y_data)

        time = frame*self.time_step
        self.time_text.set_text("Time (sec): %.1f" % time)

        if self.wait_for_input:
            raw_input("")

        return self.animated_objects

    def run(self):
        """Start animation
        """
        self.n_frame = self.agents[-1].states.shape[1]

        anim = FuncAnimation(self.fig, self.animate, init_func=self.init_animation,
                             frames=self.n_frame, interval=(self.time_step*1000*self.slow_rate),
                             blit=True)

        if SAVE_ANIMATION:
            anim.save('path_planning_demo.mp4', fps=int(1/self.time_step),
                      extra_args=['-vcodec', 'libx264'])

        plt.show()

    def plot_obstacle(self, obstacles):
        "Plot obstacle"
        for each_obstacle in obstacles:
            x_data = []
            y_data = []
            for coord in each_obstacle:
                x_data.append(coord[0])
                y_data.append(coord[1])

            self.axes.plot(x_data, y_data, c='k', alpha=1, lw=5, marker='o')
