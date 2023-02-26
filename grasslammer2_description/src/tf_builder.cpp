#include <memory>
#include <sstream>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
using std::placeholders::_1;


class TF_builder : public rclcpp::Node
{
  public:
    TF_builder()
    : Node("tf_builder")
    {
      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/grasslammer_velocity_controller/odom", 1, std::bind(&TF_builder::topic_callback, this, _1));

      tf_1_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      ground_truth_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth", 10, std::bind(&TF_builder::gt_callback, this, _1)
      );

      tf_2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);




    }
     

  private:
    void topic_callback(const nav_msgs::msg::Odometry & msg) const
    {
        //create a transform stamped t to populate
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg.header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";

        t.transform.translation.x = msg.pose.pose.position.x;
        t.transform.translation.y = msg.pose.pose.position.y;
        t.transform.translation.z = msg.pose.pose.position.z;

        t.transform.rotation.x = msg.pose.pose.orientation.x;
        t.transform.rotation.y = msg.pose.pose.orientation.y;
        t.transform.rotation.z = msg.pose.pose.orientation.z;
        t.transform.rotation.w = msg.pose.pose.orientation.w;


        // Send the transformation
        tf_1_broadcaster_->sendTransform(t);
      
    }
    void gt_callback(const nav_msgs::msg::Odometry & msg) const
    {
        //create a transform stamped t to populate
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg.header.stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "ground_truth";

        t.transform.translation.x = msg.pose.pose.position.x;
        t.transform.translation.y = msg.pose.pose.position.y;
        t.transform.translation.z = msg.pose.pose.position.z;

        t.transform.rotation.x = msg.pose.pose.orientation.x;
        t.transform.rotation.y = msg.pose.pose.orientation.y;
        t.transform.rotation.z = msg.pose.pose.orientation.z;
        t.transform.rotation.w = msg.pose.pose.orientation.w;


        // Send the transformation
        tf_2_broadcaster_->sendTransform(t);
      
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_1_broadcaster_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_2_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF_builder>());
  rclcpp::shutdown();
  return 0;
}
