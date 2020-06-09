#!/usr/bin/env python
"""Module to plot the trajectories of agents.

Circles represent the agents, dashed line the predicted trajectory over the horizon

"""

# import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
plt.style.use('seaborn-pastel')

class TrajPlot(object):
    """To plot trajectories of agents
    """
    def __init__(self, agent_list, time_step):
        """Init

        Args:
            x (array): Trajectories
            h (float): Time step
        """
        self.agents = agent_list # Position and acceleration at each time step
        self.n_agents = len(self.agents)
        self.time_step = time_step # Time step

        #: int: Number of frame, corresponds to number of column
        self.n_frame = self.agents[0].positions_data.shape[1]

        self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.fig.set_size_inches(7, 7)
        self.axe = plt.axes(xlim=(-0.5, 5), ylim=(-0.5, 5))
        self.axe.set_title('Trajectories')
        self.axe.set_xlabel('x (m)')
        self.axe.set_ylabel('y (m)')

        self.color_list = ['b', 'r', 'g', 'c', 'm', 'y', 'k']
        self.animated_objects = [] # List of all objects to animate
        self.init_animated_objects()

    def init_animated_objects(self):
        """Creates all objects to animate.

        Each agent has a circle (current position) and dashed line (predicted trajectory)

        Notes:
            Structure of animated object. Idx:
                0: circle of agent 1
                1: line of agent 1
                2: circle of agent 2
                3: line of agent 2
                ...
                -1: time text

        """
        for _, color in zip(self.agents, self.color_list):
            circle = Circle((0, 0), 0.15, alpha=0.8, fc=color)
            line, = self.axe.plot([], [], lw=2, linestyle='dashed', color=color)

            self.axe.add_patch(circle)

            self.animated_objects.append(circle)
            self.animated_objects.append(line)


        # Add time_text
        self.time_text = self.axe.text(0.02, 0.95, '', transform=self.axe.transAxes)
        self.animated_objects.append(self.time_text)

    def init_animation(self):
        """Initialize animation
        """
        for i in range(self.n_agents):
            agent = self.agents[i]

            # Circle
            self.animated_objects[2*i].center = (agent.positions_data[0, 0],
                                                 agent.positions_data[1, 0])

            # Line
            self.animated_objects[2*i+1].set_data([], [])

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
            data = agent.positions_data[:, frame]

            # Circle
            self.animated_objects[2*i].center = (data[0], data[1])

            x_data = []
            y_data = []
            z_data = []

            for k in range(int(len(data)/6)):
                x_data.append(data[6*k])
                y_data.append(data[6*k + 1])
                z_data.append(data[6*k + 2])

            self.animated_objects[2*i + 1].set_data(x_data, y_data)

        time = frame*self.time_step
        self.time_text.set_text("Time (sec): %.1f" % time)

        return self.animated_objects

    def run(self):
        """Start animation
        """
        _ = FuncAnimation(self.fig, self.animate, init_func=self.init_animation,
                          frames=self.n_frame, interval=(self.time_step*1000), blit=True)

        plt.show()


def plot_traj(agent_list, time_step):
    """Plot trajectrorie

    Args:
        agent_list ([type]): List of agents
        time_step ([type]): Time step (sec)
    """
    traj_plot = TrajPlot(agent_list, time_step)
    traj_plot.run()
