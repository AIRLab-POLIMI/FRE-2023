#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <limits>
#include <utility>
#include <functional>


class cloud_to_scan_ransac : public rclcpp::Node{
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;


    public:
        cloud_to_scan_ransac() : Node("cloud_to_scan_ransac"){

            this->declare_parameter("size", 5000); //size of coud
            
            this->declare_parameter("max_range", 3.0); //max laser range

            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ransac_final_cloud", 10, std::bind(&cloud_to_scan_ransac::callback, this, std::placeholders::_1));
                        
            pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_final_ransac", 1);

        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){     
            const int size = this->get_parameter("size").as_int();
            const float max_range = this->get_parameter("max_range").as_double();
            pcl::PCLPointCloud2 pc2; //intermediate pointcloud transformations
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //input cloud obtained from /selected            

            //Conversion from input pointcloud2 to pcl_cloud_in PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud_in);   

            auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_msg->header = input->header;

            scan_msg->angle_min = -M_PI;
            scan_msg->angle_max = M_PI;
            scan_msg->angle_increment = M_PI/size * 2;
            scan_msg->time_increment = 0.0;
            scan_msg->scan_time = pcl_cloud_in->header.stamp;
            scan_msg->range_min = 0.0;
            scan_msg->range_max = max_range;

            scan_msg->ranges.assign(size, std::numeric_limits<double>::infinity());
            //scan_msg->ranges.assign(size, max_range);

            // Iterate through pointcloud
            pcl::PointXYZ point;
            for (long unsigned int i=0; i < pcl_cloud_in->points.size(); i++) {
                point=pcl_cloud_in->points[i];

                double range = hypot(point.x, point.y);
                double angle = atan2(point.y, point.x);
                bool flag;

                // overwrite range at laserscan ray if new range is smaller
                int index = (angle + M_PI)/(scan_msg->angle_increment);
                if (range < scan_msg->ranges[index]) {
                    flag = false;
                    if(range < 0.9 && index > size/8 && index < size*7/8 && !(index > size*3/8 && index < size*5/8)){
                        for(int i=0; i<28; i++){
                            if((index-i)>=0 && (range >= scan_msg->ranges[index-i] + 0.075) && (range <= scan_msg->ranges[index-i] + 0.40)){
                                flag = true;
                                i=80;
                            }
                            if((index+i)>=0 && (range >= scan_msg->ranges[index+i] + 0.075) && (range <= scan_msg->ranges[index-i] + 0.40)){
                                flag = true;
                                i=80;
                            }
                        }
                    }
                    if(!flag)scan_msg->ranges[index] = range;
                }
            }
            pub->publish(std::move(*scan_msg));      
        }

};



int main(int argc, char **argv)
{
    srand (time(NULL));
    rclcpp::init(argc, argv);
 	rclcpp::spin(std::make_shared<cloud_to_scan_ransac>());
    rclcpp::shutdown();
 	return 0;
}
