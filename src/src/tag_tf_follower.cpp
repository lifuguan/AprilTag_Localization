#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

void method_one(ros::NodeHandle nh)
{
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/pose", 10);
    float dst = 1.0;
    tf::StampedTransform transform, transform_target;
    geometry_msgs::Twist vel_msg;
    geometry_msgs::Pose pose_msg;

    tf::TransformBroadcaster target_frame_broadcaster, orbit_frame_broadcaster;
    tf::TransformListener orbit_camera_listener, target_camera_listener;
    ros::Rate rate(10.0);

    while (nh.ok())
    {
        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 1.05)),
                                                                    ros::Time::now(), "tag_0", "target_point"));

        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, dst)),
                                                                    ros::Time::now(), "target_point", "orbit_point"));

        try
        {
            //get the relative position (on rviz)
            orbit_camera_listener.lookupTransform("/camera", "/orbit_point", ros::Time(0), transform);
            target_camera_listener.lookupTransform("/camera", "/target_point", ros::Time(0), transform_target);
            ROS_DEBUG("Acutal position : [ x =  %f, y = %f ]", transform_target.getOrigin().z(), transform_target.getOrigin().x());
            pose_msg.position.x = transform_target.getOrigin().z();
            pose_msg.position.y = transform_target.getOrigin().x();
            pub_pose.publish(pose_msg);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        if (transform_target.getOrigin().z() <= 0.05)
        {
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0;
            ROS_DEBUG("ARRIVED THE TARGET.");
        }
        else
        {
            vel_msg.angular.z = 1.0 * atan2(transform.getOrigin().x(), transform.getOrigin().z());
            // vel_msg.linear.x = 1.0 * sqrt(pow(transform.getOrigin().z(), 2) + pow(transform.getOrigin().x(), 2));
            vel_msg.linear.x = dst;

            ROS_DEBUG("angular : %f, linear : %f", vel_msg.angular.z, vel_msg.linear.x);
        }

        if (transform.getOrigin().z() <= dst)
        {
            if (dst <= 0.001)
            {
                dst = 0.01;
            }
            else
            {
                dst /= 2;
            }
        }
        else if (transform.getOrigin().z() > 1.0)
        {
            dst = 1.0;
        }
        pub.publish(vel_msg);
        rate.sleep();
    }
}

void method_two(ros::NodeHandle nh)
{
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    float dst = 0.0;
    double theta = 0;
    tf::StampedTransform transform_target;
    tf::TransformBroadcaster target_frame_broadcaster;
    tf::TransformListener target_camera_listener;
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        //target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(1.57, 1.57, 0.0), tf::Vector3(0.0, 0.0, 1.05)), ros::Time::now(), "tag_0", "target_point"));
        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0.0, 0.0, 0.0), tf::Vector3(0.0, 0.0, 6)), ros::Time::now(), "odom", "target_point"));
        double roll, pitch, yaw, angle;
        try
        {
            //get the relative position (on rviz)
            target_camera_listener.lookupTransform("/camera_link", "/odom", ros::Time(0), transform_target);

            tf::Matrix3x3(transform_target.getRotation()).getEulerYPR(yaw, pitch, roll);
            angle = yaw / M_PI * 180;
            dst = sqrt(pow(transform_target.getOrigin().x(), 2) + pow(transform_target.getOrigin().y(), 2));

            ROS_INFO("DST : %f ; ANGLE : %f", dst, angle);
            vel_msg.linear.x = 0.8 * tanh(3.8 * dst);
            if (transform_target.getOrigin().y() == 0)
            {
                theta = 0;
            }
            
            else if (transform_target.getOrigin().y() < 0)
            {
                theta = M_PI_2 - atan(transform_target.getOrigin().x() / transform_target.getOrigin().y() * (-1));
            }
            else
            {
                theta = -M_PI_2 + atan(transform_target.getOrigin().x() / transform_target.getOrigin().y());
            }

            vel_msg.angular.z = (7 * theta) + ((-1) * yaw);

            ROS_INFO("theta : %f ; yaw : %f ; angular : %f; linear : %f", theta, yaw,
                     vel_msg.angular.z,
                     vel_msg.linear.x);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
        }
        // vel_msg.linear.x = 0;
        // vel_msg.angular.z =0;
        pub.publish(vel_msg);
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_tf_follower");
    ros::NodeHandle nh;

    //method_one(nh);
    method_two(nh);

    return 0;
}
