#include <ros/ros.h>
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
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub2;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filter;
        double roll, pitch, yaw, init_roll, init_pitch, init_yaw;
        bool flag = false; //flag for initializing Imu data
        double eps_angle = 8.0; //maximum deviation from the axis specified in perpendicular plane
        double threshold = 0.12; //distance threshold for selecting the points near the plane
        Eigen::VectorXf coefficients; //found coefficient of the plane
        pcl::Indices inliers; //inliers for the found plane
        
    public:
        plane_filter() : Node("plane_filter"){
            //n = rclcpp::Node::make_shared("plane_filter");
            
            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/velodyne_points2", 10, std::bind(&plane_filter::callback_lidar, this, std::placeholders::_1));
            
            sub2 = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&plane_filter::callback_imu, this, std::placeholders::_1));
            
            pub_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/selected", 1);
    }

        void callback_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
            sensor_msgs::msg::PointCloud2 cloud_msg; //Message for containing the pointcloud to be published
            pcl::PCLPointCloud2 pc2, pc2_out;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr appo_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

            //Minimum and maximum values for the cropbox filter.
            //x, y, z and intensity
            Eigen::Vector4f min_pt (-3.0f, -3.0f, -3.0f, -1.0f);
            Eigen::Vector4f max_pt (3.0f, 3.0f,  3.0f, 1.0f);
            //Define the matrix to rotate the pointcloud using pitch data from the imu
            Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
            Eigen::Matrix3f m;

            //Conversion from input pointcloud2 to pcl_cloud PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud);

            m = Eigen::AngleAxisf(pitch,  Eigen::Vector3f::UnitY());        
            transform_2.rotate(m);
        
            //Executing the rotation
            pcl::transformPointCloud (*pcl_cloud, *transformed_cloud, transform_2);         
            
        
            //Filtering with a cropbox filter the pointcloud rotated using imu data
            pcl::CropBox<pcl::PointXYZ> cropBoxFilter (false); //set to false to not return deleted points
            cropBoxFilter.setInputCloud (transformed_cloud);
            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);
            //Cloud filtering
            cropBoxFilter.filter (*cloud_filtered);
            

            pcl::IndicesPtr new_inliers(new pcl::Indices);
            pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr
                model_p (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (cloud_filtered));

            if (inliers.size()) {
                //selects the points near the latest coefficients (plane of iteration t-1)
                //and stores them in new_inliers
                model_p->selectWithinDistance(coefficients, threshold, *new_inliers);

                //Extracts the part of the pointcloud previously selected
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(new_inliers);
                extract.setNegative(false); //returns the points near the plane
                extract.filter(*appo_cloud); //save the selected point for further processing
                
                //Define a new Perpendicular plane model over the selected cloud.
                //The ransac will find the new coefficient using the points selected with the
                //coefficients found at t-1
                pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ>::Ptr
                    model_p2 (new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> (appo_cloud));
                //Set the axis to which the plane has to be perpendicular to
                model_p2->setAxis(Eigen::Vector3f::UnitZ());
                //Sets the maximum angle from which the found plane can deviate
                //it is a mandatory parameter for perpendicular plane selection to work
                model_p2->setEpsAngle(pcl::deg2rad(eps_angle));
                //Defines ransac over the perpendicular plane model. 
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p2);
                ransac.setDistanceThreshold (threshold);
                ransac.computeModel();
                coefficients.conservativeResize(4);
                ransac.getModelCoefficients(coefficients); //store new coefficients
                //uses the new coefficients to select the points of the pointcloud at time t
                //belonging to the plane
                model_p->selectWithinDistance(coefficients, threshold, *new_inliers);
                inliers = *new_inliers;
            }
            else {
                //Executed only at the first iteration for finding the first plane
                model_p->setAxis(Eigen::Vector3f::UnitZ()); 
                model_p->setEpsAngle(pcl::deg2rad(eps_angle));
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
                ransac.setDistanceThreshold (threshold);
                ransac.computeModel(); 
                ransac.getInliers(*new_inliers);
                coefficients.conservativeResize(4);
                ransac.getModelCoefficients(coefficients); //save the current coefficient
                inliers = *new_inliers; //saves the inliers
            }

            if (new_inliers->size () == 0)
            {
                PCL_ERROR("Could not estimate a planar model for the given dataset.");
            }
            else {
                //Extracting indices for filtering the pointcloud
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(cloud_filtered);
                extract.setIndices(new_inliers);
                extract.setNegative(true); //true to remove the plane, false to see the plane
                extract.filter(*output_cloud);
            }

            //Conversion from PCLPointCloud<T> to PCLPointCloud2 and from PCLPointCloud2 to sensor_msgs::msg::PointCloud2
            pcl::toPCLPointCloud2(*output_cloud, pc2_out);
            pcl_conversions::fromPCL(pc2_out, cloud_msg);
            pub_filter->publish(cloud_msg);
        }

        void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& input){
            tf2::Quaternion tf_quat(input->orientation.x, input->orientation.y, input->orientation.z,
                input->orientation.w);
            tf2::Matrix3x3 m(tf_quat);
            //getting roll pitch yaw from quaternion
            m.getRPY(roll, pitch, yaw);
            //initializing the data for later adjusting the new readings
            if (!flag) {
                init_roll = roll;
                init_pitch = pitch;
                init_yaw = yaw;
                flag = true;
            }
            roll = roll - init_roll;
            pitch = pitch - init_pitch;
            yaw = yaw - init_yaw;
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
