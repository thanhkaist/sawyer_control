#!/usr/bin/env python

"""

@author: aim lab
@email: cd_yoo@kaist.ac.kr
"""

import rospy
from intera_interface.limb import Limb
from intera_io import IODeviceInterface
from geometry_msgs.msg import Pose
import moveit_commander

import copy

from sawyer_control.srv import grasping


def execute_grasp(data):
    rospy.loginfo('one request')

    if gripper_io.get_signal_value("is_calibrated") is not True:
        gripper_io.set_signal_value("calibrate", True)

    if data.req:
        rospy.loginfo('Close')
        endpoint_pose = arm.endpoint_pose()
        pose_target = Pose()
        pose_target.position.x = endpoint_pose['position'].x
        pose_target.position.y = endpoint_pose['position'].y
        pose_target.position.z = endpoint_pose['position'].z
        pose_target.orientation.x = endpoint_pose['orientation'].x
        pose_target.orientation.y = endpoint_pose['orientation'].y
        pose_target.orientation.z = endpoint_pose['orientation'].z
        pose_target.orientation.w = endpoint_pose['orientation'].w

        # Make the gripper pully wide open, getting ready to grab
        gripper_io.set_signal_value("position_m", 0.041)
        rospy.sleep(1)

        waypoints = []
        wpose = pose_target
        waypoints.append(copy.deepcopy(wpose))
        for i in range(0, 3):
            wpose.position.z -= 0.065
            waypoints.append(copy.deepcopy(wpose))

        (grip_plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)  ## eef_step, jump_threshold

        group.execute(grip_plan)
        rospy.sleep(1)

        # grabbing bottle to a fairly tight position, and check how much force it's sensing right now
        gripper_io.set_signal_value("speed_mps", 1)
        gripper_io.set_signal_value("position_m", 0.00)
        light_size = gripper_io.get_signal_value("position_response_m")
        rospy.sleep(1)
    else:
        # grabbing bottle to a fairly tight position, and check how much force it's sensing right now
        rospy.loginfo('Open')
        gripper_io.set_signal_value("speed_mps", 1)
        gripper_io.set_signal_value("position_m", 0.041)
        light_size = gripper_io.get_signal_value("position_response_m")
        rospy.sleep(1)

    return True


def grasp_server():
    rospy.init_node('grasp_server', anonymous=True)

    global arm
    arm = Limb()

    global group
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(0.0005)
    group.set_max_acceleration_scaling_factor(0.0005)
    group.set_planning_time(20)

    global gripper_io
    gripper_io = IODeviceInterface("end_effector", 'right_gripper')

    s = rospy.Service('grasping', grasping, execute_grasp)
    rospy.spin()


if __name__ == "__main__":
    grasp_server()
