#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

void transformPoint(const tf::TransformListener& listener)
{
    geometry_msgs::PointStamped camera_point;
    camera_point.header.frame_id = "camera";
    camera_point.header.stamp = ros::Time();

    try
    {
        geometry_msgs::PointStamped tag_point;
        listener.transformPoint("tag_0", camera_point, tag_point);
        ROS_INFO("camera_point : [%f, %f, %f] -> tag_point  : [%f, %f, %f]",  camera_point.point.x, camera_point.point.y, camera_point.point.z, tag_point.point.x, tag_point.point.y, tag_point.point.z);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"tag_point\" to \"camera_point\": %s", ex.what());
    }
    
}


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "tag_tf_listener");
    ros::NodeHandle nh;
     
    // tf::TransformListener listener(ros::Duration(10));
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
 
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform("/tag_0", "/camera", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ROS_INFO("camera_point -> tag_point  : [%f, %f, %f]", 
         transform.getOrigin().x(),  transform.getOrigin().y(), transform.getOrigin().z());
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                        transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                    pow(transform.getOrigin().y(), 2));

        rate.sleep();
    }
    return 0;
}