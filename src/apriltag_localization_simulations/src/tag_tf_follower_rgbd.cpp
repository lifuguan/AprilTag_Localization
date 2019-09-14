#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

static const std::string IMAGE_TOPIC = "/kinect_ir/kinect/depth/points";
static const std::string PUBLISH_TOPIC = "/pcl/points";

ros::Publisher pub;
sensor_msgs::PointCloud2 output;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filltered;

    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    // 点云稀疏
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filltered);

    pcl_conversions::moveFromPCL(cloud_filltered, output);

    // ROS msg to PointXYZ
    pcl::PointCloud<pcl::PointXYZ> icp_cloud_;
    pcl::fromROSMsg(output, icp_cloud_);

    pub.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_tf_follower_rgbd");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Initalize ROS Node : " << ros::this_node::getName());

    ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    // while (nh.ok())
    // {
    //     ROS_INFO_STREAM("PointCloud :" << output.data);
    // }

    ros::spin();
    return 0;
}