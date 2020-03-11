#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from sawyer_control.srv import *


DEFAULT_TARGET_POSITION = [0.5,0.5,0.5]

class VisualTargetServer():
    def __init__(self):
        self.node = rospy.init_node('VisualTargetServer',anonymous=True)
        self.__pub = rospy.Publisher('/my_target',PointStamped,queue_size=10)
        self.__service = rospy.Service('set_target',set_target,self.service_callbacK)
        self.current_pos = DEFAULT_TARGET_POSITION
        for i in range(5):
            self.pub_point(DEFAULT_TARGET_POSITION)

    def run(self):
        rospy.spin()

    def service_callbacK(self,res):
        rospy.loginfo(res)
        for i in range(5):
            self.pub_point(res.point)

        self.current_pos = res.point
        return set_targetResponse(True)


    def pub_point(self,point):
        point_stampe = PointStamped()
        point_stampe.header.frame_id="base"
        point_stampe.point.x = point[0]
        point_stampe.point.y = point[1]
        point_stampe.point.z = point[2]
        self.__pub.publish(point_stampe)

if __name__ == '__main__':
    visual_target_server = VisualTargetServer()
    visual_target_server.run()