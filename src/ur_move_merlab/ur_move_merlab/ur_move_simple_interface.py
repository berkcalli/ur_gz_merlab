#!/usr/bin/env python3
# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Felix Exner

# This is an example of how to interface the robot without any additional ROS components. For
# real-life applications, we do recommend to use something like MoveIt!

import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from common_interfaces_merlab.srv import SendJointTrajectory
from common_interfaces_merlab.srv import SendJointTrajectoryPoint



class JTCClient(rclpy.node.Node):

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()
        self.get_logger().info(f"Action server is detected")

        self.srv = self.create_service(SendJointTrajectoryPoint, 'move_traj_single_point', self.singlePointTrajectoryCallback)
        self.srv = self.create_service(SendJointTrajectory, 'move_traj_multi_point', self.multiPointTrajectoryCallback)
        self.i = 0

    def singlePointTrajectoryCallback(self, request, response):

        self.get_logger().info("Received Single Pose service")
        self.execute_setpoint(request.goal_point)
        response.success = True
        return response

    def multiPointTrajectoryCallback(self, request, response):

        self.get_logger().info("Received Trajectory with multiple set points")
        self.execute_multi_point_trajectory(request.goal_points)
        response.success = True
        return response
    


    def execute_setpoint(self, trajectory_point):
        self.get_logger().info(f"Executing one setpoint")
        goal = JointTrajectory()
        goal.joint_names = self.joints
        point = JointTrajectoryPoint()
        point = trajectory_point
 
        goal.points.append(point)

        goal_to_execute = FollowJointTrajectory.Goal()
        goal_to_execute.trajectory = goal

        goal_to_execute.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal_to_execute.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal_to_execute)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        

    def execute_multi_point_trajectory(self, trajectory_points):
        self.get_logger().info("Executing multiple setpoint")
        goal = JointTrajectory()
        goal.joint_names = self.joints

        for point_x in trajectory_points.points:
            print('Point positions:', point_x.positions)
            print('Point time_from_start:', point_x.time_from_start)

            goal.points.append(point_x)

            goal_to_execute = FollowJointTrajectory.Goal()
            goal_to_execute.trajectory = goal

            goal_to_execute.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
            goal_to_execute.goal_tolerance = [
            JointTolerance(position=0.02, velocity=0.02, name=self.joints[i]) for i in range(6)
            ]

        self._send_goal_future = self._action_client.send_goal_async(goal_to_execute)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"

def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    try:
        rclpy.spin(jtc_client)
    except RuntimeError as err:
        jtc_client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")

    rclpy.shutdown()

if __name__ == "__main__":
       main()
