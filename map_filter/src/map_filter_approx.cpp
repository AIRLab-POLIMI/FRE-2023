#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

#include <sensor_msgs/msg/laser_scan.hpp>

typedef struct neighborData{
    short int here;
    short int neighbors;
}neighborData;


class map_filter_approx : public rclcpp::Node{
    private:

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scanner;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filter;



        sensor_msgs::msg::LaserScan laser_scan;

    public:
        map_filter_approx() : Node("map_filter_approx"){

            this->declare_parameter("threshold", 120); //minimum number of neighbors

            this->declare_parameter("dynamic_range", 50.0); //maximum scaling factor due to distance

            this->declare_parameter("precision",0.0375);  //size of a unit measure. Decreasing it increase quadratically the performance but increase precision

            this->declare_parameter("height", 8.0); //height of the considered point cloud

            this->declare_parameter("width", 8.0);  //width of the considered point cloud

            this->declare_parameter("laserscan_range", 1.5); //range in which points from monoplanar lidar are added

            this->declare_parameter("biased", true);   //if biased we sligtly favours points in having the same orientation as the robot

            this->declare_parameter("max_threshold", 200); //maximum threshold for valid points


            sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/selected", 10, std::bind(&map_filter_approx::callback, this, std::placeholders::_1));

            sub_scanner = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_initial", 10, std::bind(&map_filter_approx::populate_scan, this, std::placeholders::_1));
                        
            pub_filter = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point_cloud", 1);

        }

        void populate_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input){
            this->laser_scan=*input;
        }

        void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input){
            pcl::PCLPointCloud2 pc2, pc2_out; //intermediate pointcloud transformations
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_in(new pcl::PointCloud<pcl::PointXYZ>); //input cloud obtained from /selected
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_out(new pcl::PointCloud<pcl::PointXYZ>); //output cloud

            

            //Conversion from input pointcloud2 to pcl_cloud_in PointCloud
            pcl_conversions::toPCL(*input, pc2);
            pcl::fromPCLPointCloud2(pc2, *pcl_cloud_in);   
            
            
            //setting header information for the output pointcloud
            pcl_cloud_out->header.frame_id = pcl_cloud_in->header.frame_id;
            pcl_cloud_out->header.stamp = pcl_cloud_in->header.stamp;

            const int width = this->get_parameter("width").as_double()/this->get_parameter("precision").as_double();
            const int height = this->get_parameter("height").as_double()/this->get_parameter("precision").as_double();
            const int size = height*width;

            //create a matrix that discretises the space. Each cell will contain the number of points in it
            neighborData zero;
            zero.here = 0;
            zero.neighbors = 0;
            neighborData * matrix = (neighborData *) malloc(sizeof(neighborData)*size);
            for(int i = 0; i < size; i++){
                matrix[i] = zero;
            }
            count_adjaciency(pcl_cloud_in, height, width, matrix); //cropped_cloud
            
            //apply the density based filter
            density_filter(pcl_cloud_in, pcl_cloud_out, width, height, matrix);

            //add point from monoplanar lidar
            add_scan_points(pcl_cloud_out);

            publish_point_cloud(pcl_cloud_out);
            //std::cout<<pcl_cloud_in->header.stamp<<" - "<<rclcpp::Time::now()<<"\n";
        }

        void count_adjaciency( const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const int height, const int width, neighborData output[]){
            pcl::PointXYZ cur;
            double precision = this->get_parameter("precision").as_double();

            for (long unsigned int i=0; i < input->points.size(); i++){
                cur=input->points[i];
                double r = pow(cur.x, 2)+pow(cur.y,2);
                double alpha = atan2(cur.x,cur.y)*180/M_1_PI;
                
                if(r > 0.45 || (alpha>-130 && alpha<130)){
                    int row = cur.x/precision + height/2;
                    int col = cur.y/precision + width/2;
                    if(row < height && col < width && row >= 0 && col>=0)output[row*width + col].here++;
                }
            }           

        }

        void density_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const pcl::PointCloud<pcl::PointXYZ>::Ptr output, const int width, const int height, neighborData matrix[]){
            int dynamic_threshold;
            double dynamic_range = this->get_parameter("dynamic_range").as_double();
            int threshold = this->get_parameter("threshold").as_int();
            double precision = this->get_parameter("precision").as_double();
            bool biased = this->get_parameter("biased").as_bool();
            int max_threshold = this->get_parameter("max_threshold").as_int();
            pcl::PointXYZ cur;

            
            for(int i = 0; i < (int)input->size(); i++){

                cur = input->points[i];
                int row = cur.x/precision + height/2;
                int col = cur.y/precision + width/2;
                double r = pow(sqrt(pow(cur.x, 2) + pow(cur.y, 2)),3);
                dynamic_threshold = std::min((int)(threshold/std::min(r, dynamic_range)), max_threshold);

                if(row > 1 && row<width-2 && height>1 && row<height-2){
                    short neighbors = matrix[row*width + col].neighbors;
                    if(neighbors == 0){
                        if(biased){
                            for(int x = - 1; x < 2; x++){
                                for(int y = -2; y < 3; y++){
                                    if(!(x!=0 && (y==-2 || y==2))) neighbors+=matrix[(row+y)*width + col+x].here;
                                }
                            }
                        }else{
                            for(int x = - 2; x < 3; x++){
                                for(int y = -2; y < 3; y++){
                                    if((x!=y && x!=-y) || (x!=-2 && x!=2)){
                                        neighbors+=matrix[(row+y)*width + col+x].here;
                                    }
                                }
                            }
                        }
                        matrix[row*width + col].neighbors = neighbors;
                    }
                    if(neighbors > dynamic_threshold) output->push_back(pcl::PointXYZ(cur.x, cur.y, 0));
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
            double y;
            int length = laser_scan.ranges.size();

            for(i=0; i<length; i++){
                th=th + laser_scan.angle_increment;
                if(laser_scan.ranges[i]<=laserscan_range){
                    y = laser_scan.ranges[i]*sin(th);
                    if((y > 0.22 && y < 0.7)||(y < -0.22 && y > -0.7)){
                        pcl_cloud->push_back(pcl::PointXYZ(laser_scan.ranges[i]*cos(th), y, 0));
                    }
                }
            }
        }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 	rclcpp::spin(std::make_shared<map_filter_approx>());
    rclcpp::shutdown();
 	return 0;
}
