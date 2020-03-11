#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from sawyer_control.srv import *


def set_target_to(point):
    rospy.wait_for_service('set_target')
    try:
        execute_action = rospy.ServiceProxy('set_target',set_target,persistent=True)
        ret = execute_action(point)

        return ret.data
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    point = [1.0,1.0,0.0]
    print(set_target_to(point))

    