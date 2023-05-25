#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <limits>
#include <utility>
#include <functional>


class cloud_to_scan : public rclcpp::Node{
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;


    public:
        cloud_to_scan() : Node("cloud_to_scan"){

            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 10, std::bind(&cloud_to_scan::callback, this, std::placeholders::_1));
                        
            pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_out", 1);

        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){     
            pcl::PCLPointCloud2 pc2; //intermediate pointcloud transformations
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //input cloud obtained from /selected            

            //Conversion from input pointcloud2 to pcl_cloud_in PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud_in);   

            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();;
            scan_msg->header = input->header;

            scan_msg->angle_min = -M_PI;
            scan_msg->angle_max = M_PI;
            scan_msg->angle_increment = M_PI/720;
            scan_msg->time_increment = 0.0;
            scan_msg->scan_time = pcl_cloud_in->header.stamp;
            scan_msg->range_min = 0.0;
            scan_msg->range_max = std::numeric_limits<double>::infinity();
            // modified
            scan_msg->ranges.assign(3240, std::numeric_limits<double>::infinity());

            // Iterate through pointcloud
            pcl::PointXYZ point;
            for (long unsigned int i=0; i < pcl_cloud_in->points.size(); i++) {
                point=pcl_cloud_in->points[i];

                double range = hypot(point.x, point.y);
                double angle = atan2(point.y, point.x);

                // overwrite range at laserscan ray if new range is smaller
                int index = (angle + M_PI)/(scan_msg->angle_increment);
                if (range < scan_msg->ranges[index]) {
                    scan_msg->ranges[index] = range;
                }
            }
            pub->publish(std::move(*scan_msg));      
        }

};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 	rclcpp::spin(std::make_shared<cloud_to_scan>());
    rclcpp::shutdown();
 	return 0;
}
