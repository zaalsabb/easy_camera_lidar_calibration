#include "lidar2pcd.hpp"

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZI>);

void Lidar2PCD::cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud_lidar(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud_lidar);

    *cloud_map += *temp_cloud_lidar;   

}

void Lidar2PCD::imageCallback (const sensor_msgs::Image& msg)
{    
    img_cv = cv_bridge::toCvCopy(msg, msg.encoding)->image;
}

void Lidar2PCD::compressedImageCallback (const sensor_msgs::CompressedImage& msg)
{    
    img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
}

void Lidar2PCD::saveData(){

    if (cloud_map->size()>0){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_map_rgb->points.resize(cloud_map->size());
        for (size_t i = 0; i < cloud_map->points.size(); i++) {
            cloud_map_rgb->points[i].x = cloud_map->points[i].x;
            cloud_map_rgb->points[i].y = cloud_map->points[i].y;
            cloud_map_rgb->points[i].z = cloud_map->points[i].z;
            cloud_map_rgb->points[i].r = 255 - cloud_map->points[i].intensity * 255;        
            cloud_map_rgb->points[i].g = 255 - cloud_map->points[i].intensity * 255;        
            cloud_map_rgb->points[i].b = 255 - cloud_map->points[i].intensity * 255;        
        }

        pcl::io::savePCDFileBinary(pcd_file, *cloud_map_rgb);

    }

    try {
        cv::imwrite(image_file,img_cv);
    } catch(cv::Exception) {
        return;
    }

}

int main (int argc, char** argv){

    ROS_INFO("Node started");

    ros::init (argc, argv, "lidar2PCD");

    ros::NodeHandle nh;

    Lidar2PCD lidar2PCD(nh);
    cout << "Press Ctrl+C to save pcd and image files..." <<endl;

    while (ros::ok()) {
        ros::spinOnce();
    }

    lidar2PCD.saveData();
    cout << "pcd and image files saved!" <<endl;

    ros::spin();
    return 0;

}
