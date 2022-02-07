#!/usr/bin/env python

from __future__ import print_function

import time
import rospy
from std_srvs.srv import Empty


def call_scan_routine():
    rospy.wait_for_service('mmm_processed_pointCloud')
    try:
        scan_routine = rospy.ServiceProxy('mmm_processed_pointCloud', Empty)
        print("a" + str(time.time()))
        scan_routine()
        print("b" + str(time.time()))
        return 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print(call_scan_routine())