#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.srv = self.create_service(
            GetMotionPlan, 'plan_service', self.plan_cb)
        self.get_logger().info('MotionPlanner ready')

    def plan_cb(self, req, resp):
        client = self.create_client(GetMotionPlan, '/plan_service')
        client.wait_for_service()
        proxy = GetMotionPlan.Request()
        proxy.motion_plan_request = req.motion_plan_request
        future = client.call_async(proxy)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        resp.error_code.val = resp.error_code.PLANNING_FAILED
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()