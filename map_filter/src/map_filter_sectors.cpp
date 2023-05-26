/*
    ros2 node that read from /selected topic a series of PointCloud2 points and then publish a new PointCloud2 on the /filtered_point_cloud topic.
    Firstly the pointcloud is filtered in order to obtain a stripe of it centered at scanner_height, with width of 2*height_threshold and parallele to the plane obtained from
    /plane topic.
    The returned pointcloud is then obtained by applying a density based filter on the cropped PointCloud2: the points are selected iff they have a number of neighbors higher then 
    a threshold. A neighbor of a point is a point whose distance from it is lower then the radius parameter. 
    The distance is computed without taking in account the z axis.
*/


#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/msg/laser_scan.hpp>

#include <vector>
#include <omp.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

typedef struct pointData{
    double x;
    double y;
    double r;    
}pointData;


class map_filter_sectors : public rclcpp::Node{
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_plane;        
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scanner;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filter;



        sensor_msgs::msg::LaserScan laser_scan;
        tf2::Quaternion plane;

    public:
        map_filter_sectors() : Node("map_filter_sectors"){

            this->declare_parameter("radius", 0.10); //radius for calculate local density

            this->declare_parameter("height_threshold", 0.08); //height of the considered area of the pointcloud

            this->declare_parameter("scanner_height", -0.17); //height of the considered area of the pointcloud


            this->declare_parameter("threshold", 40); //minimum number of neighbors

            this->declare_parameter("dynamic_range", 15.0); //maximum scaling factor due to distance

            this->declare_parameter("laserscan_range", 1.5); //range in which points from monoplanar lidar are added


            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/selected", 10, std::bind(&map_filter_sectors::callback, this, std::placeholders::_1));

            sub_scanner = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&map_filter_sectors::populate_scan, this, std::placeholders::_1));

            sub_plane = this->create_subscription<geometry_msgs::msg::Quaternion>("/plane", 10, std::bind(&map_filter_sectors::get_plane, this, std::placeholders::_1));
                        
            pub_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 1);

        }

        void populate_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input){
            this->laser_scan=*input;
        }

        void get_plane(const geometry_msgs::msg::Quaternion input){
            tf2::fromMsg(input, this->plane);
        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
            pcl::PCLPointCloud2 pc2; //intermediate pointcloud transformations
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //input cloud obtained from /selected
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); //output cloud

            

            //Conversion from input pointcloud2 to pcl_cloud_in PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud_in);   
            
            
            //setting header information for the output pointcloud
            pcl_cloud_out->header.frame_id = pcl_cloud_in->header.frame_id;
            pcl_cloud_out->header.stamp = pcl_cloud_in->header.stamp;

            //order cloud points inside array based on angle and distance
            const int size = 360;
            //each vector ordered_points[alpha] will represent an array of points which relative angle from the lidar is alpha-180 degrees
            std::vector<pointData> ordered_points[size]; 
            //this takes all the points from the pointcloud and place them ordinately inside ordered_points
            //ordered_points[i] will now be a vectors of points ordered by the distance from the lidar
            order_cloud_points(pcl_cloud_in, ordered_points, size);

            //apply the density based filter
            density_filter(ordered_points, pcl_cloud_out, size);

            //add point from monoplanar lidar
            add_scan_points(pcl_cloud_out);

            //transform pcl_cloud_out to a PCLPointCloud2 and then publish it
            publish_point_cloud(pcl_cloud_out);
        }

        void order_cloud_points( const pcl::PointCloud<pcl::PointXYZ>::Ptr input, std::vector<struct pointData> output[], const int size){
            pcl::PointXYZ cur;
            int alpha;
            double r;
            pointData converted;
            double height_threshold = this->get_parameter("height_threshold").as_double();
            double height = this->get_parameter("scanner_height").as_double();


            for (long unsigned int i=0; i < input->points.size(); i++){
                cur=input->points[i];
                
                alpha = atan2(cur.y, cur.x)*(180/M_PI) + 180;
                
                r = sqrt(pow(cur.x,2) + pow(cur.y,2));
                
                converted = {cur.x, cur.y, r};
                //remove the points belonging to the robot itself
                if(r > 0.7 || (alpha > 30 && alpha < 330)){
                    //consider only the point near the ground plane
                    double d = (std::abs(plane[0])/plane[0])*(plane[0]*cur.x + plane[1]*cur.y + plane[2]*cur.z);
                    if(d < height + height_threshold && d > height - height_threshold){
                        output[alpha].push_back(converted);
                    }
                }
            }
            //sort the points using quicksort
            #pragma omp parallel for
            for(int i = 0; i < size; i++){
                quicksort_pointData(output[i], 0, (int) (output[i].size() - 1));
            }
        }

        int partition_pointData(std::vector<struct pointData> &values, int left, int right) {
            int pivotIndex = left + (right - left) / 2;
            double pivotValue = values[pivotIndex].r;
            int i = left, j = right;
            pointData temp;
            while(i <= j) {
                while(values[i].r < pivotValue) {
                    i++;
                }
                while(values[j].r > pivotValue) {
                    j--;
                }
                if(i <= j) {
                    temp = values[i];
                    values[i] = values[j];
                    values[j] = temp;
                    i++;
                    j--;
                }
            }
            return i;

        }

        void quicksort_pointData(std::vector<struct pointData> &values, int left, int right) {
            if(left < right) {
                int pivotIndex = partition_pointData(values, left, right);
                quicksort_pointData(values, left, pivotIndex - 1);
                quicksort_pointData(values, pivotIndex, right);
            } 
        }

        //returns the closest value in the array that is bigger than value
        int binary_search(const std::vector<struct pointData> vector, double value, int start, int end){
            int cur = (end + start)/2;
            if (cur == start) return start;
            if (value >= vector[end].r) return end;
            if (value <= vector[start].r) return start;

            if(vector[cur].r >= value){
                if(vector[cur-1].r <= value){
                    int x = vector[cur].r - value;
                    int y = value - vector[cur -1].r;
                    if(x < y) return cur;
                    return cur - 1;
                }
                return binary_search(vector, value, start,  cur - 1);                
            }
            return binary_search(vector, value, cur + 1, end);  
        }

        void density_filter(const std::vector<struct pointData> input[], const pcl::PointCloud<pcl::PointXYZ>::Ptr output, const int size){
            pcl::PointXYZ converted;    
            int dynamic_threshold;
            int dynamic_alpha;
            double radius = this->get_parameter("radius").as_double();
            double dynamic_range = this->get_parameter("dynamic_range").as_double();
            int threshold = this->get_parameter("threshold").as_int();
            double scale_factor;
            int neighbors;
            pointData selected;
            double rsqrd = pow(radius,2);

            
            for(int i = 0; i < size; i++){


                for(int j = 0; j < (int) (input[i].size()); j++){

                    selected = input[i][j];
                    scale_factor = pow(selected.r, 2);

                    if(scale_factor > dynamic_range){
                        dynamic_threshold = threshold/dynamic_range;
                    }
                    else{
                        dynamic_threshold = threshold/scale_factor;
                    } 
                    dynamic_alpha =(int) (180/M_PI)*(atan(1.1*radius/selected.r)); //considers a slightli bigger sector for reducing errors
                    

                    neighbors = 0;
                    double start_x = selected.x;
                    double start_y = selected.y;

                    #pragma omp parallel for reduction(+:neighbors)
                    for(int k = -dynamic_alpha; k<=dynamic_alpha; k++){
                        int temp;
                        int cur_angle = i + k;
                        if(cur_angle<0) cur_angle+=360;
                        if(cur_angle>=360) cur_angle-=360;

                        int cur_length = input[cur_angle].size();

                        int lowerBound = binary_search(input[cur_angle], selected.r - radius, 0, cur_length -1);
                        int higherBound = binary_search(input[cur_angle], selected.r + radius, 0, cur_length -1);
                        temp = higherBound - lowerBound;
                        int mid = lowerBound + temp/2;
                        
                        for(int h = lowerBound; h < mid; h++){
                            pointData target = input[cur_angle][h];
                            if(pow(target.x-start_x,2) + pow(target.y-start_y,2)>rsqrd) temp--;
                            else h=mid;
                        }
                        for(int h = higherBound; h > mid; h--){
                            pointData target = input[cur_angle][h];
                            if(pow(target.x-start_x,2) + pow(target.y-start_y,2)>rsqrd) temp--;
                            else h=mid;
                        }
                        neighbors+=temp;
                        if(neighbors > dynamic_threshold) k=dynamic_alpha;
                    }
                    if(neighbors >= dynamic_threshold){
                        pcl::PointXYZ point;
                        point.x = selected.x;
                        point.y = selected.y;
                        point.z  = 0.0;
                        output->push_back(point);
                    }
                }
            }
        }

        //publish a point cloud including the latest update from the laserscan
        void publish_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud){
            sensor_msgs::msg::PointCloud2 cloud_msg; //Message for containing the pointcloud to be published
            pcl::PCLPointCloud2 pc2;

            //Conversion from PCLPointCloud<T> to PCLPointCloud2 and from PCLPointCloud2 to sensor_msgs::msg::PointCloud2
            pcl::toPCLPointCloud2(*pcl_cloud, pc2);
            pcl_conversions::fromPCL(pc2, cloud_msg);
            //publish the pcl_cloudnew pointcloud
            pub_filter->publish(cloud_msg);
        }

        //add nearby points from laserscan
        void add_scan_points(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud){
            double laserscan_range = this->get_parameter("laserscan_range").as_double();
            int i;
            float th=laser_scan.angle_min;
            int length = laser_scan.ranges.size();

            for(i=0; i<length; i++){
                th=th + laser_scan.angle_increment;
                if(laser_scan.ranges[i]<=laserscan_range){
                    //remove the points belonging to the robot back
                    if(laser_scan.ranges[i]>0.65 || (th > -2.22 && th < 2.22) || (th > -2.62 && th < 2.62 && laser_scan.ranges[i] > 0.4)){
                        pcl_cloud->push_back(pcl::PointXYZ(laser_scan.ranges[i]*cos(th), laser_scan.ranges[i]*sin(th), 0));
                    }
                }
            }
        }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 	rclcpp::spin(std::make_shared<map_filter_sectors>());
    rclcpp::shutdown();
 	return 0;
}