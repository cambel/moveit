#!/usr/bin/env python

import unittest
import numpy as np
import rospy
import rostest
import os

# from moveit_ros_planning_interface._moveit_move_group_interface import (
#     MoveGroupInterface,
# )
import moveit_commander

from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory


class RobotStateUpdateTest(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        self.rocket_group = moveit_commander.MoveGroupCommander("rocket")

    @classmethod
    def tearDown(self):
        pass

    def test(self):
        initial_config = [1.566859, -1.566849, 1.256859, -1.563712, -1.563721, 0.0]
        self.rocket_group.go(joints=initial_config)

        rocket_pose = self.rocket_group.get_current_pose("rocket_tool0")
        # print(rocket_pose)
        rocket_pose.pose.position.x += 0.05
        rocket_pose.pose.position.y += 0.05
        rocket_pose.pose.position.z -= 0.10
        
        self.rocket_group.set_start_state_to_current_state()
        self.rocket_group.set_pose_target(rocket_pose, end_effector_link="rocket_tool0")

        result = self.rocket_group.plan()
        if isinstance(result, RobotTrajectory):
            self.assertTrue(result.joint_trajectory.joint_names)
            self.rocket_group.execute(result)
        else:
            success, plan, planning_time, error = result
            self.assertTrue(error.val == MoveItErrorCodes.SUCCESS)
            self.rocket_group.execute(plan)

        goal_joint_configuration = [1.415227, -1.458163, 1.613571, -2.024163, -1.519018, -0.144994]

        print("trac-ik solution:", np.round(self.rocket_group.get_current_joint_values(), 4).tolist())
        print("expected solution:", np.round(goal_joint_configuration, 4).tolist())

        self.assertTrue(np.allclose(goal_joint_configuration, self.rocket_group.get_current_joint_values(), 0.001))


if __name__ == "__main__":
    PKGNAME = "moveit_ros_planning_interface"
    NODENAME = "moveit_test_robot_state_update"
    rospy.init_node(NODENAME)
    rostest.rosrun(PKGNAME, NODENAME, RobotStateUpdateTest)

    # suppress cleanup segfault in ROS < Kinetic
    os._exit(0)
