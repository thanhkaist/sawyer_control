#!/usr/bin/env python

"""
This script sets up a service server and gets data from the ar_pose_marker
topic and then return it to the service client

@author: aim lab
@email: cd_yoo@kaist.ac.kr
"""

import rospy

from ar_track_alvar_msgs.msg import AlvarMarkers
from sawyer_control.srv import target, targetResponse


class ObjectLocation():
    def __init__(self, marker_id=0):
        self.marker_id = marker_id
        self.p = None

    def callback(self, data):
        # get the msg AlvarMarkers
        ar_marker = data
        markers = ar_marker.markers
        # get the msg Pose which has position and orientation information on it
        self.p = []
        obj = None
        for i in range(len(markers)):
            if markers[i].id == self.marker_id:
                obj = markers[i]
        if obj is not None:
            self.p.append(obj.pose.pose.position.x)
            self.p.append(obj.pose.pose.position.y)
            self.p.append(obj.pose.pose.position.z)
        return

    def handle_gettarget(self, req):
        print("Returning target position")
        # create a subscriber to get the ar poss imformation from /ar_pose_marker topic
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        # return the ar tag pose information to the service client
        return targetResponse(self.p)

    def gettarget_server(self):
        rospy.init_node('locate_object_server')
        rospy.Service('locate_object', target, self.handle_gettarget)
        rospy.spin()


if __name__ == "__main__":
    marker_id_want_to_track = 6
    v = ObjectLocation(marker_id=marker_id_want_to_track)
    v.gettarget_server()
