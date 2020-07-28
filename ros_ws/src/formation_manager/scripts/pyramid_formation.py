#!/usr/bin/env python

"""Pyramid formation
"""

from math import sin, pi, ceil
import rospy

from crazyflie_driver.msg import Position

from general_formation import FormationClass, compute_info_from_center


class PyramidFormation(FormationClass):
    """Pyramid formation

    Notes:
        scale: Height of the pyramid
        swarm_pose: Same as position of CF 0

    Layouts:

        y
        |
        |
        |_____x

        ------ Top view -----
        2       3

            0

        1       4

        ------ Top view -----
        6               7

            2       3

                0

            1       4

        5               8

        ------ Side view -----
                0

            1       4

        5               8

    """
    def __init__(self, min_dist):
        super(PyramidFormation, self).__init__(min_dist)

        self._min_height = self._scale + 0.5

        # Attrs specific to square
        self.n_tier = 0 #: (int) Number of tier in the Pyramid
        self.tier_dist = 0 #: (float) Distance between each "tier" of the pyramid. Tier 0 at the top
        self.theta = 45*pi/180 #: (float) Angle between 1-0-4 (see side view)

        self.compute_min_scale()

    # Setter
    def set_n_agents(self, n_agents):
        # Verify number of CFs, n-1 must be a multiple of 4
        if n_agents > 0 and (n_agents  - 1) % 4 == 0:
            self._n_agents = n_agents
            self._n_agents_landed = 0
        else:
            self._n_agents_landed = (n_agents  - 1) % 4
            self._n_agents = n_agents - self._n_agents_landed
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" %\
                self._n_agents_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self._n_agents)
        self.find_extra_agents()

        self.update_formation_scale()
        self.compute_min_scale()

    # Computing
    def compute_min_scale(self):
        min_scale_tier_dist = self._min_dist*self.n_tier if self.n_tier > 0 else 0
        min_scale_ag_dist = self._min_dist/(2*sin(self.theta))*self.n_tier

        self._min_scale = min(min_scale_tier_dist, min_scale_ag_dist)

    def compute_formation_positions(self):
        # (dX, dY) sign for each position in tier
        tier_poses_sign = [(-1, -1), (-1, 1), (1, 1), (1, -1)]

        for i in range(self._n_agents):
            if rospy.is_shutdown():
                break

            # Initialize agent formation goal
            self._agents_goals[i] = Position()

            # Find tier information
            # i=0 -> tier=0, i=1,2,3,4 -> tier = 1, i=5,6,7,8 -> tier = 2 ...
            tier_num = ceil(i/4.0)
            tier_pos = i%4 # Position in the tier
            square_length = 2*sin(self.theta)*self.tier_dist*tier_num

            # Compute formation position
            z_dist = -1 * tier_num * self.tier_dist

            x_dist = tier_poses_sign[tier_pos][0]*square_length/2
            y_dist = tier_poses_sign[tier_pos][1]*square_length/2

            # information from center
            center_dist, theta, center_height = compute_info_from_center([x_dist, y_dist, z_dist])
            self._center_dist[i] = center_dist
            self._angle[i] = theta
            self._center_height[i] = center_height

    def update_formation_scale(self):
        self.n_tier = (self._n_agents - 1) / 4
        self._min_height = self._scale + 0.5 if self.n_tier > 1 else 0.5

        # Space between tiers
        self.tier_dist = self._scale/self.n_tier if self.n_tier > 0 else 0
