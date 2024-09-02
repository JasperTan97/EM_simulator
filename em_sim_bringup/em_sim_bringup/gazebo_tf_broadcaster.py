"""
Reads pose message as published in /model/{robot_name}/tf
Publishes tf with parent {robot_name}/odom and child {robot_name}/base_link
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class GazeboTfBroadcaster(Node):

    def __init__(self):
        super().__init__('gazebo_tf_broadcaster')
        self.declare_parameter('robot_name', 'robot_unnamed')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.subscription = self.create_subscription(
            PoseArray,
            f'/model/{self.robot_name}/tf',
            self.listener_callback,
            10
        )
        self.subscription

        self.br = TransformBroadcaster(self)

    def listener_callback(self, msg):
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{self.robot_name}/odom'  # Parent frame
        t.child_frame_id = f'{self.robot_name}/base_link'  # Use the robot name

        # Convert pose to transform
        t.transform.translation.x = msg.poses[0].position.x
        t.transform.translation.y = msg.poses[0].position.y
        t.transform.translation.z = msg.poses[0].position.z
        t.transform.rotation = msg.poses[0].orientation

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()