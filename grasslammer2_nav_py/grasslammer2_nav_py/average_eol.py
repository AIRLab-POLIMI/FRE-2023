import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler



class AverageBroadcaster(Node):
    # assume type are exponentials
    def __init__(self) :
        super().__init__('average_broadcaster')
        self.start_average = 0
        self.end_average = 0
        self.index = 0

        self.calculate_average = self.create_subscription(PoseStamped, '/end_of_line_pose', self.callback, 1)

        # publish once end of line pose
        self.end_of_line_pose_topic_pub = self.create_publisher(PoseStamped, '/final_pose', 1)

    
    def calculate_average(self, msg):
        end_pose = PoseStamped()
        end_pose.header = msg.header
        end_pose.pose = msg.pose
        (roll, pitch, yaw) = euler_from_quaternion(end_pose.pose.orientation.x, end_pose.pose.orientation.y, end_pose.pose.orientation.z, end_pose.pose.orientation.w)
        if(self.index%2 == 0):
            self.start_average += yaw/((int) (self.index + 1)/2)    
            self.publish_end_of_line_pose(end_pose.pose.position.x, end_pose.pose.position.y, self.start_average)
        else:
            self.end_average += yaw/((int) (self.index + 1)/2)     
            self.publish_end_of_line_pose(end_pose.pose.position.x, end_pose.pose.position.y, self.end_average)

        self.index += 1
        


    # publish end of line pose
    def publish_end_of_line_pose(self, x, y, theta):
        # create message Pose
        end_of_line_pose = PoseStamped()
        
        # update timestamp and frame
        time_now = Time()
        end_of_line_pose.header.stamp = time_now.to_msg()
        end_of_line_pose.header.frame_id = "odom"

        # get x,y
        end_of_line_pose.pose.position.x = x
        end_of_line_pose.pose.position.y = y

        # get orientation
        quaternion = quaternion_from_euler(0, 0, theta)
        end_of_line_pose.pose.orientation.x = quaternion[0]
        end_of_line_pose.pose.orientation.y = quaternion[1]
        end_of_line_pose.pose.orientation.z = quaternion[2]
        end_of_line_pose.pose.orientation.w = quaternion[3]

        self.end_of_line_pose_topic_pub.publish(end_of_line_pose)

def main(args=None):
    rclpy.init(args=args)

    average_broadcaster = AverageBroadcaster()

    rclpy.spin(average_broadcaster)

    average_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
