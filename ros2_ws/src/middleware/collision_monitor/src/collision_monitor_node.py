#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collision_monitor.msg import CollisionAlert

class CollisionMonitor(Node):
    """
    Subscribes to /joint_states and /servo_feedback,
    checks for excessive effort/torque, and publishes CollisionAlert.
    """
    def __init__(self):
        super().__init__('collision_monitor')
        # Subscribe to joint_states (hardware & sim)
        self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_cb,
            10)
        # Publisher for alerts
        self._alert_pub = self.create_publisher(
            CollisionAlert,
            'collision_alert',
            10)
        self.get_logger().info('CollisionMonitor initialized')

    def joint_state_cb(self, msg: JointState):
        # Example: use effort array if present
        if msg.effort:
            for name, effort in zip(msg.name, msg.effort):
                if abs(effort) > 5.0:
                    alert = CollisionAlert()
                    alert.joint_name = name
                    alert.value = effort
                    alert.description = 'effort limit exceeded'
                    self._alert_pub.publish(alert)
                    self.get_logger().warn(
                        f"CollisionAlert: {name} effort={effort:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()