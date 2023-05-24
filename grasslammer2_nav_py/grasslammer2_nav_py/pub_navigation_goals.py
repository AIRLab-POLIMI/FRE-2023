import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped




class GoalPubblisher(Node):

    def __init__(self):
        super().__init__('pub_goal_pose')
        #self.position = self.create_subscription()
        self.goalPubblisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.pub_goal_pose()
    


    def pub_goal_pose(self):
        
        #TODO ADJUST AND COMPILE THE POSITION IN BASE OF WHERE WE ARE
        time_now = Time()
        poseToNavigate = PoseStamped()

        poseToNavigate.header.stamp = time_now.to_msg()
        poseToNavigate.header.frame_id = "map"

        poseToNavigate.pose.position.x = -4.0
        poseToNavigate.pose.position.y = 3.0
        poseToNavigate.pose.position.z = 0.0

        poseToNavigate.pose.orientation.x = 0.0
        poseToNavigate.pose.orientation.y = 0.0
        poseToNavigate.pose.orientation.z = 0.0
        poseToNavigate.pose.orientation.w = 1.0
        
        sleep(5)
        self.goalPubblisher.publish(poseToNavigate)


    
def main(args=None):
    rclpy.init(args=args)
    node = GoalPubblisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
