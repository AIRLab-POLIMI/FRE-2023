import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class tfEndOfLine(Node):

    def __init__(self):
        super().__init__('fix_frame_end_of_line')
        self.sub = self.create_subscription(PoseStamped, '/end_of_line_pose', self.update_tf, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
    

    def update_tf(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'end_of_line_pose'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.0

        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'end_of_line_pose'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.x
        t.transform.translation.z = msg.pose.position.x
        t.transform.rotation.x = msg.pose.position.x
        t.transform.rotation.y = msg.pose.position.x
        t.transform.rotation.z = msg.pose.position.x
        t.transform.rotation.w = msg.pose.position.x

        self.tf_broadcaster.sendTransform(t)
        