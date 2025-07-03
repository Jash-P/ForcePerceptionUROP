#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from control_msgs.action import GripperCommand
from application_layer.action import Grasp

class GraspManager(Node):
    def __init__(self):
        super().__init__('grasp_manager')
        # Action server
        self._action_server = ActionServer(
            self, Grasp, 'grasp',
            self.execute_cb, self.goal_cb, self.cancel_cb)
        # Clients
        self._move_cli = self.create_client(MoveGroup, 'move_action')
        self._move_cli.wait_for_service()
        self._grip_cli = self.create_client(GripperCommand, 'gripper_action')
        self._grip_cli.wait_for_service()
        self.get_logger().info('GraspManager ready')

    def goal_cb(self, request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = Grasp.Feedback()
        # Pre-grasp path
        move_goal = MoveGroup.Goal()
        move_goal.request.group_name = 'manipulator'
        # TODO: build constraints from goal.target_pose
        move_handle = self._move_cli.send_goal_async(move_goal)
        result = await move_handle.get_result_async()
        if result.status != result.GoalStatus.STATUS_SUCCEEDED:
            goal_handle.abort()
            return Grasp.Result(success=False, message='pre-grasp failed')
        feedback.stage = 'pre-grasp'
        goal_handle.publish_feedback(feedback)
        # Gripper
        grip_goal = GripperCommand.Goal()
        grip_goal.command.position = goal.width/2.0
        grip_goal.command.max_effort = 10.0
        grip_handle = self._grip_cli.send_goal_async(grip_goal)
        res = await grip_handle.get_result_async()
        if res.status != res.GoalStatus.STATUS_SUCCEEDED:
            goal_handle.abort()
            return Grasp.Result(success=False, message='gripper failed')
        feedback.stage = 'gripper closed'
        goal_handle.publish_feedback(feedback)
        # Lift & retreat omitted
        goal_handle.succeed()
        return Grasp.Result(success=True, message='grasp succeeded')

def main(args=None):
    rclpy.init(args=args)
    node = GraspManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()