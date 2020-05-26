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
        self._emergency_list = []
        for each_cf in cf_list:
            self.crazyflies.append(Crazyflie(each_cf))
            rospy.loginfo("waiting for emergency service of " + each_cf)
            rospy.wait_for_service('/' + each_cf + '/emergency')
            rospy.loginfo("found emergency service of " + each_cf)
            self._emergency_list.append(rospy.ServiceProxy('/' + each_cf + '/emergency', srv.Empty))


        # Launch services
        rospy.Service('/update_params', srv.Empty, self.update_params)
        rospy.Service('/emergency', srv.Empty, self.emergency)
        rospy.Service('/stop', srv.Empty, self.stop)
        rospy.Service('/takeoff', srv.Empty, self.takeOff)   
        rospy.Service('/land', srv.Empty, self.land)      

    # Services methods
    def update_params(self, req):
        print("UPDATE PARAMS")
        return srv.EmptyResponse()

    def emergency(self, req):
        rospy.logerr("EMERGENCY")
        for each_emergency in self._emergency_list:
            each_emergency()

        return srv.EmptyResponse()

    def stop(self, req):
        print("STOP")
        for cf in self.crazyflies: cf.stop_trig()
        return srv.EmptyResponse()
    
    def takeOff(self, req):
        print("TAKE OFF")
        for cf in self.crazyflies: cf.take_off_trig()
        return srv.EmptyResponse()
    
    def land(self, req):
        print("LAND")
        for cf in self.crazyflies: cf.land_trig()
        return srv.EmptyResponse()
        
if __name__ == '__main__':
    rospy.init_node('swarmManager', anonymous=False)
    rospy.loginfo('Initialisation du swarm manager')
    cf_list = rospy.get_param("~cf_list", "['cf1']")
    swarm = Swarm(cf_list)

    
    rospy.spin()