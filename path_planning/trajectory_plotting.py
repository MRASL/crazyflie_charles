#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
plt.style.use('seaborn-pastel')

class TrajPlot:
    def __init__(self, x, h):
        self.x = x # Position and acceleration at each time step
        self.h = h # Time step
        self.n_frame = x.shape[1]

        self.fig = plt.figure()
        self.fig.set_dpi(100)
        self.fig.set_size_inches(7, 7)
        self.ax = plt.axes(xlim=(-0.5, 5), ylim=(-0.5, 5))
        self.ax.set_title('Trajectories')
        self.ax.set_xlabel('x (m)')
        self.ax.set_ylabel('y (m)')


        self.line, = self.ax.plot([], [], lw=2, linestyle='dashed', color='b')

        self.circle = Circle((0, 0), 0.15, fc='b', alpha=0.8)
        self.ax.add_patch(self.circle)

        self.time_text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)

    def init_animation(self):
        self.line.set_data([], [])
        self.circle.center = (self.x[0, 0], self.x[1, 0])
        self.time_text.set_text('')

        return [self.circle, self.line, self.time_text]

    def animate(self, i):
        data = self.x[:, i]
        self.circle.center = (data[0], data[1])

        x_data = []
        y_data = []
        z_data = []

        for k in range(int(len(data)/6)):
            x_data.append(data[6*k])
            y_data.append(data[6*k + 1])
            z_data.append(data[6*k + 2])

        self.line.set_data(x_data, y_data)

        time = i*self.h
        self.time_text.set_text("Time (sec): %.1f" % time)

        return [self.line, self.circle, self.time_text]

    def run(self):
        anim = FuncAnimation(self.fig, self.animate, init_func=self.init_animation,
                                    frames=self.n_frame, interval=(self.h*1000), blit=True)

        plt.show()
    

def plot_traj(x, h):
    traj_plot = TrajPlot(x, h)

    traj_plot.run()
    
