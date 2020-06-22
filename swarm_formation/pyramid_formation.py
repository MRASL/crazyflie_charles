#!/usr/bin/env python

"""Square formation
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
    def __init__(self, offset=None):
        if offset is None:
            offset = [0, 0, 0]
        super(PyramidFormation, self).__init__(offset=offset)

        self.min_scale = 0.5

        # Attrs specific to square
        self.n_tier = 0 #: (int) Number of tier in the Pyramid
        self.tier_dist = 0 #: (float) Distance between each "tier" of the pyramid. Tier 0 at the top
        self.theta = 45*pi/180 #: (float) Angle between 1-0-4 (see side view)

    # Setter
    def set_n_agents(self, n_agents):
        # Verify number of CFs, n-1 must be a multiple of 4
        if n_agents > 0 and (n_agents  - 1) % 4 == 0:
            self.n_agents = n_agents
            self.n_agents_landed = 0
        else:
            self.n_agents_landed = (n_agents  - 1) % 4
            self.n_agents = n_agents - self.n_agents_landed
            rospy.loginfo("Formation: Unsuported number of CFs, landing %i CF" %\
                self.n_agents_landed)

        rospy.loginfo("Formation: %i crazyflies in formation" % self.n_agents)
        self.land_extra_agents()

        self.n_tier = (self.n_agents - 1) / 4

        if self.n_tier != 0:
            self.tier_dist = self.scale/self.n_tier

    # Computing
    def compute_start_positions(self, formation_goal):
        center = [formation_goal.x,
                  formation_goal.y,
                  formation_goal.z]

        # (dX, dY) sign for each position in tier
        tier_poses_sign = [(-1, -1), (-1, 1), (1, 1), (1, -1)]

        for i in range(self.n_agents):
            if rospy.is_shutdown():
                break
            agent_goal = Position()
            agent_goal.yaw = 0

            # Find tier information
            # i=0 -> tier=0, i=1,2,3,4 -> tier = 1, i=5,6,7,8 -> tier = 2 ...
            tier_num = ceil(i/4.0)
            tier_pos = i%4 # Position in the tier
            square_length = 2*sin(self.theta)*self.tier_dist*tier_num

            # Find goals

            z_pose = center[2] - tier_num * self.tier_dist

            x_dist = tier_poses_sign[tier_pos][0]*square_length/2
            y_dist = tier_poses_sign[tier_pos][1]*square_length/2

            agent_goal.x = center[0] + x_dist
            agent_goal.y = center[1] + y_dist
            self.agents_goals[i] = agent_goal

            # Find distances from center
            center_dist, theta, center_height =\
                compute_info_from_center([agent_goal.x, agent_goal.y, z_pose],
                                         center)
            self.center_dist[i] = center_dist
            self.angle[i] = theta
            self.center_height[i] = center_height

        return self.agents_goals

    def update_scale(self, formation_goal):
        # Space between CFs
        self.tier_dist = self.scale/self.n_tier if self.n_tier > 0 else 0

        self.compute_start_positions(formation_goal)
