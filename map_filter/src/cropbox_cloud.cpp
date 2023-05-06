#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

#include <pcl/ModelCoefficients.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>


class plane_filter : public rclcpp::Node{
    private:
        //rclcpp::Node::SharedPtr n;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filter;
        
    public:
        plane_filter() : Node("plane_filter"){
            
            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points2", 10, std::bind(&plane_filter::callback_lidar, this, std::placeholders::_1));
                        
            pub_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_cloud", 1);
    }

        void callback_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
            sensor_msgs::msg::PointCloud2 cloud_msg; //Message for containing the pointcloud to be published
            pcl::PCLPointCloud2 pc2, pc2_out;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud);


            //Minimum and maximum values for the cropbox filter.
            //x, y, z and intensity
            Eigen::Vector4f min_pt (-2.0f, -1.5f, -0.6f, -1.0f);
            Eigen::Vector4f max_pt (2.0f, 1.5f, 0.6f, 1.0f); 
            
        
            //Filtering with a cropbox filter the pointcloud rotated using imu data
            pcl::CropBox<pcl::PointXYZ> cropBoxFilter (false); //set to false to not return deleted points
            cropBoxFilter.setInputCloud (pcl_cloud);
            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);
            //Cloud filtering
            cropBoxFilter.filter (*cloud_filtered);
            
            pcl::toPCLPointCloud2(*cloud_filtered, pc2_out);
            pcl_conversions::fromPCL(pc2_out, cloud_msg);
            
            pub_filter->publish(cloud_msg);
        }

};



int main(int argc, char **argv)
{
    //ros::init(argc, argv, "talker");
    rclcpp::init(argc, argv);
 	//plane_filter plane_filter;
 	rclcpp::spin(std::make_shared<plane_filter>());
    
 	return 0;
}
