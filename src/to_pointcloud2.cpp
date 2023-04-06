#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

ros::Publisher pub;
ros::Subscriber sub;
typedef pcl::PointXYZINormal PointType;

void   horizon_handler( const livox_ros_driver::CustomMsg::ConstPtr &msg );
void   pub_func( pcl::PointCloud< PointType > &pl, ros::Publisher pub, const ros::Time &ct );

int main( int argc, char **argv )
{
    ros::init( argc, argv, "to_pointcloud2" );
    ros::NodeHandle n;
    pub = n.advertise< sensor_msgs::PointCloud2 >( "/livox/lidar_pc2", 100 );

    sub = n.subscribe( "/livox/lidar", 1000, horizon_handler, ros::TransportHints().tcpNoDelay() );

    ros::spin();
    return 0;
}

void pub_func( pcl::PointCloud< PointType > &pl, ros::Publisher pub, const ros::Time &ct )
{
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl, output );
    output.header.frame_id = "livox";
    output.header.stamp = ct;
    pub.publish( output );
}


void horizon_handler( const livox_ros_driver::CustomMsg::ConstPtr &msg )
{
    pcl::PointCloud< PointType >           pl_full;

    uint plsize = msg->point_num;

    pl_full.resize( plsize );

    for ( uint i = 1; i < plsize; i++ )
    {

        pl_full[ i ].x = msg->points[ i ].x;
        pl_full[ i ].y = msg->points[ i ].y;
        pl_full[ i ].z = msg->points[ i ].z;
        pl_full[ i ].intensity = msg->points[ i ].reflectivity;

    }    
    
    pub_func( pl_full, pub, msg->header.stamp );
}


