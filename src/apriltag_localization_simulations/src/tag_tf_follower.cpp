#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_tf_follower");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate rate(10.0);
    tf::TransformBroadcaster target_frame_broadcaster, orbit_frame_broadcaster;
    tf::TransformListener orbit_camera_listener, target_camera_listener;

    float dst = 1.0;
    while (nh.ok())
    {
        tf::StampedTransform transform, transform_target;
        geometry_msgs::Twist vel_msg;

        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.05)),
        ros::Time::now(),"tag_0", "target_point"));

        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, dst)),
            ros::Time::now(),"target_point", "orbit_point"));
        
        try
        {
            //get the relative position (on rviz)
            orbit_camera_listener.lookupTransform("/camera", "/orbit_point", ros::Time(0), transform);
            target_camera_listener.lookupTransform("/camera", "/target_point", ros::Time(0), transform_target);
            ROS_INFO("Acutal position : [ x =  %f, y = %f ]", transform_target.getOrigin().z(), transform_target.getOrigin().x());
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        if (transform_target.getOrigin().z() <= 0.05)
        {
            vel_msg.linear.x = 0.0; vel_msg.angular.z = 0;
            ROS_INFO("ARRIVED THE TARGET.");
        }
        else
        {
            vel_msg.angular.z = 1.0 * atan2(transform.getOrigin().x(), transform.getOrigin().z());
            // vel_msg.linear.x = 1.0   * sqrt(pow(transform.getOrigin().z(), 2) + pow(transform.getOrigin().x(), 2));
            vel_msg.linear.x = dst;

            ROS_DEBUG("angular : %f, linear : %f", vel_msg.angular.z, vel_msg.linear.x);
        }
        
        if (transform.getOrigin().z() <=dst)
        {
            if (dst <= 0.001)
            {
                dst = 0.01;
            }
            else
            {
                dst/=2;
            }
        }
        else if (transform.getOrigin().z() > 1.0)
        {
            dst = 1.0;
        }
    
        pub.publish(vel_msg);
        rate.sleep();
    }

    return 0;
}