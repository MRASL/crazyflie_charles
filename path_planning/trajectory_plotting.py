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
        self.circle = Circle((0, 0), 0.15, fc='b', alpha=0.8)


    def init_animation(self):
        self.circle.center = (self.x[0, 0], self.x[1, 0])
        self.ax.add_patch(self.circle)
        return self.circle,

    def animate(self, i):
        self.circle.center = (self.x[0, i], self.x[1, i])

        return self.circle,


    def run(self):
        anim = FuncAnimation(self.fig, self.animate, init_func=self.init_animation,
                                    frames=self.n_frame, interval=(self.h*1000), blit=True)

        plt.show()
    

def plot_traj(x, h):
    traj_plot = TrajPlot(x, h)

    traj_plot.run()
    
