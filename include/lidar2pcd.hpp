// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
// opencv header files
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
// pcl header files
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
// #include <pcl/filters/voxel_grid.h> 

using namespace std;

class Lidar2PCD {

    public:

        Lidar2PCD(ros::NodeHandle nh){
            // Initialize the node
            nh_ = nh;
          
            ros::param::get( "PointCloudTopic", pcd_topic );            
            ros::param::get( "ImageTopic", img_topic );                                

            ros::param::get( "common/image_file", image_file );   
            ros::param::get( "common/pcd_file", pcd_file );

            sub = nh_.subscribe (pcd_topic, 1, &Lidar2PCD::cloudCallback, this);

            if (img_topic.find("/compressed") != string::npos){
                sub2 = nh_.subscribe (img_topic, 1, &Lidar2PCD::compressedImageCallback, this);
            } else {
                sub2 = nh_.subscribe (img_topic, 1, &Lidar2PCD::imageCallback, this);
            }
           

        };

        ros::NodeHandle nh_;

        string pcd_topic;
        string img_topic;

        string pcd_file;
        string image_file;        

        ros::Subscriber sub;
        ros::Subscriber sub2;

        cv::Mat img_cv;

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
        void imageCallback(const sensor_msgs::Image& image);
        void compressedImageCallback (const sensor_msgs::CompressedImage& msg);

        void saveData();

};
