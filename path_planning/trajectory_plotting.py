#!/usr/bin/env python
"""Module to plot the trajectories of agents.

Circles represent the agents, dashed line the predicted trajectory over the horizon

"""

# import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
plt.style.use('seaborn-pastel')

SAVE_ANIMATION = False

class TrajPlot(object):
    """To plot trajectories of agents
    """
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
        self.fig.set_size_inches(7, 7)
        self.axe = plt.axes(xlim=(-1, 5), ylim=(-1, 5))
        self.axe.set_title('Trajectories')
        self.axe.set_xlabel('x (m)')
        self.axe.set_ylabel('y (m)')

        self.color_list = ['b', 'r', 'g', 'c', 'm', 'y', 'k', 'b', 'r', 'g', 'c', 'm', 'y', 'k']
        self.animated_objects = [] # List of all objects to animate
        self.init_animated_objects()

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
        for each_agent, color in zip(self.agents, self.color_list):
            circle = Circle((0, 0), 0.15, alpha=0.8, fc=color)
            line, = self.axe.plot([], [], lw=2, linestyle='dashed', color=color)#, marker='o')

            self.axe.add_patch(circle)

            self.animated_objects.append(circle)
            self.animated_objects.append(line)

            # Draw goal
            x_goal = each_agent.goal[0]
            y_goal = each_agent.goal[1]
            self.axe.scatter(x_goal, y_goal, s=250, c=color, marker='X')

        # Add time_text
        self.time_text = self.axe.text(0.02, 0.95, '', transform=self.axe.transAxes)
        self.animated_objects.append(self.time_text)

    def init_animation(self):
        """Initialize animation
        """
        for i in range(self.n_agents):
            agent = self.agents[i]

            # Circle
            self.animated_objects[2*i].center = (agent.states[0, 0],
                                                 agent.states[1, 0])

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
            data = agent.states[:, frame]

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

    def plot_obstacle(self, coords):
        "Plot obstacle"
        x_data = []
        y_data = []
        for coord in coords:
            x_data.append(coord[0])
            y_data.append(coord[1])

        self.axe.plot(x_data, y_data, c='k', alpha=1, lw=5, marker='o')
