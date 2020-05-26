#!/usr/bin/env python

"""
To manage the flight of the swarm
"""

import rospy
import tf
from crazyflie import Crazyflie

from std_srvs import srv


class Swarm:
    def __init__(self, cf_list):
        self.crazyflies = []
        for each_cf in cf_list:
            self.crazyflies.append(Crazyflie(each_cf))

        # Launch services
        rospy.Service('/stop', srv.Empty, self.stop)
        rospy.Service('/takeoff', srv.Empty, self.takeOff)   
        rospy.Service('/land', srv.Empty, self.land)        

    def stop(self, req):
        print("STOP")
        return srv.EmptyResponse()
    
    def takeOff(self, req):
        print("TAKE OFF")
        return srv.EmptyResponse()
    
    def land(self, req):
        print("LAND")
        return srv.EmptyResponse()
        
if __name__ == '__main__':
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    swarm = Swarm(cf_list)

    
    rospy.spin()