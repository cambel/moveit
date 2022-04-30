#!/usr/bin/env python

import unittest
import rospy
import rostest
import os

import moveit_commander

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory


class RobotStateUpdateTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.dual_arm_group = moveit_commander.MoveGroupCommander(
            "rocket_and_groot")
        self.rocket_group = moveit_commander.MoveGroupCommander("rocket")
        self.groot_group = moveit_commander.MoveGroupCommander("groot")

    @classmethod
    def tearDown(self):
        pass

    def plan(self, target):
        self.dual_arm_group.set_joint_value_target(target)
        return self.dual_arm_group.plan()

    def test(self):
        rocket_pose = self.rocket_group.get_current_pose("rocket_tool0")
        rocket_pose.pose.position.z -= 0.05
        rocket_pose.pose.position.x += 0.05

        groot_pose = self.groot_group.get_current_pose("groot_tool0")
        groot_pose.pose.position.z -= 0.05
        groot_pose.pose.position.x -= 0.05
        self.dual_arm_group.set_start_state_to_current_state()
        self.dual_arm_group.set_pose_target(
            rocket_pose, end_effector_link="rocket_tool0")
        self.dual_arm_group.set_pose_target(
            groot_pose, end_effector_link="groot_tool0")

        result = self.dual_arm_group.plan()
        if isinstance(result, RobotTrajectory):
            self.assertTrue(result.joint_trajectory.joint_names)
        else:
            success, plan, planning_time, error = result
            self.assertTrue(error.val == MoveItErrorCodes.SUCCESS)


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_robot_state_update"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, RobotStateUpdateTest)

    # suppress cleanup segfault in ROS < Kinetic
    os._exit(0)
