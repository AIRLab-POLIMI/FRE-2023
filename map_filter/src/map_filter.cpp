/*
    ros2 node that read from /selected topic a series of PointCloud2 points and then publish a new PointCloud2 on the /filtered_point_cloud topic.
    The returned pointcloud is obtained by applying a density based filter: the points are selected iff they have a number of neighbors higher then 
    a threshold. A neighbor of a point is a point whose distance from it is lower then the radius parameter. 
    The distance is computed without taking in account the z axis.

    The node and the package are both called map_filter

    To tune the algorithm is possible to modify the parameters at lines 27 and 28
*/


//#include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

class map_filter : public rclcpp::Node{
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filter;


        double radius = 0.13; //radius for calculate local density
        int threshold = 8; //minimum number of neighbors

        
    public:
        map_filter() : Node("map_filter"){
            
            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/selected", 10, std::bind(&map_filter::callback, this, std::placeholders::_1));
                        
            pub_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 1);


        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
            sensor_msgs::msg::PointCloud2 cloud_msg; //Message for containing the pointcloud to be published
            pcl::PCLPointCloud2 pc2, pc2_out; //intermediate pointcloud transformations
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //input cloud obtained from /selected
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); //output cloud
            

            //Conversion from input pointcloud2 to pcl_cloud_in PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud_in);   
            
            //setting header information for the output pointcloud
            pcl_cloud_out->header.frame_id = pcl_cloud_in->header.frame_id;
            pcl_cloud_out->header.stamp = pcl_cloud_in->header.stamp;

            //apply the density based filter
            density_filter(pcl_cloud_in, pcl_cloud_out);

            //Conversion from PCLPointCloud<T> to PCLPointCloud2 and from PCLPointCloud2 to sensor_msgs::msg::PointCloud2
            pcl::toPCLPointCloud2(*pcl_cloud_out, pc2_out);
            pcl_conversions::fromPCL(pc2_out, cloud_msg);
            //publish the new pointcloud
            pub_filter->publish(cloud_msg);
        }

        //given a point cloud "input" selects the points having enough neighbors. Selected points are added to the "output" pointcloud
        void density_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const pcl::PointCloud<pcl::PointXYZ>::Ptr output){
            //counter for the neighbors
            int neighbors;
            //double used for saving the distance 
            double distance;
            //the considered point from the point cloud "input"
            pcl::PointXYZ point;
            //the point from the input point cloud that may be a neighbor of "point"
            pcl::PointXYZ other;

            //for every point in the "input" point cloud
            for (long unsigned int i=0; i < input->points.size(); i++) {
                point=input->points[i];
                neighbors=0;
                //for every point in the "input" point cloud
                for (long unsigned int j=0; j < input->points.size(); j++){
                    other=input->points[j];

                    //we compute the squared distance from the "other" point to "point"
                    //note that we don't use sqrt(...) because is much more computationally onerous
                    distance = pow((point.x - other.x),2) + pow((point.y - other.y),2);
                    //if the distance squared is less than the radius squared than "other" is a neighbor of "point"
                    if(distance<pow(radius,2)){
                        neighbors++;
                    }
                }
                //if current "point" has enough neighbors it is added to the output
                //note that we have ">" and not ">=" because in the neighbors is counted "point" himself
                if(neighbors>threshold){
                    output->push_back(point);
                }
            }
        }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 	rclcpp::spin(std::make_shared<map_filter>());
    
 	return 0;
}
