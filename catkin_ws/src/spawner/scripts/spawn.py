#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from turtlesim.srv import *

def spawn_function():
    rospy.wait_for_service('spawn')
    try:
        activate = rospy.ServiceProxy('spawn', Spawn)
        responce = activate(10,10,0, 'turtle2')
        return
    except rospy.ServiceException as e:
        print("service call failed: %s" %e)
        return

if __name__ == "__main__":
    spawn_function()
