#include <memory>
#include <sstream>
#include <string>
#include <functional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


#include "geometry_msgs/msg/transform_stamped.hpp"
using std::placeholders::_1;

const double TWO_PI  = 2 * M_PI;
const double PI = M_PI;

class Rotations : public rclcpp::Node // define a class called rotations that inherits from the rclpp's class Node, public means that all the public attributes remains public and the privates remains privates 
{
  public:
    double count = 0.0;
    double prev_yaw = 0.0; 

    Rotations()
    : Node("rotations")
    {
        
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/grasslammer_velocity_controller/odom", 1, std::bind(&Rotations::topic_callback, this, _1));
        // The std::bind function is used to bind the callback function topic_callback to the this object, which is an instance of the MinimalSubscriber class. The _1 argument specifies the argument that will be passed to the callback function. In this case, _1 represents a std_msgs::msg::String message.

    }
    

  private:
    
    void count_rot(double curr_yaw){
        if (sign_change(curr_yaw)){
            count+=0.5;
        }
        

    }

    bool sign_change(double curr_yaw){
        if (curr_yaw >= 0 && prev_yaw<0){
            return true;
        }
        else if (curr_yaw < 0 && prev_yaw>=0){
            return true;
        }
        return false;
    }   

    void topic_callback(const nav_msgs::msg::Odometry & msg) 
    {


        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg.header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg.pose.pose.position.x;
        t.transform.translation.y = msg.pose.pose.position.y;
        t.transform.translation.z = msg.pose.pose.position.z;

        t.transform.rotation.x = msg.pose.pose.orientation.x;
        t.transform.rotation.y = msg.pose.pose.orientation.y;
        t.transform.rotation.z = msg.pose.pose.orientation.z;
        t.transform.rotation.w = msg.pose.pose.orientation.w;

        tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        

        count_rot(yaw);
        RCLCPP_INFO(this->get_logger(), "rotations: %f \t%f",
                    count, yaw);
        
        prev_yaw = yaw; 
      
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rotations>());
  rclcpp::shutdown();
  return 0;
}