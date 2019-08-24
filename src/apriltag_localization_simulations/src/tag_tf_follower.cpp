#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>




void method_one()
{
    float dst = 1.0;
    tf::StampedTransform transform, transform_target;
    geometry_msgs::Twist vel_msg;

    tf::TransformBroadcaster target_frame_broadcaster, orbit_frame_broadcaster;
    tf::TransformListener orbit_camera_listener, target_camera_listener;
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
}

void method_two(ros::NodeHandle nh)
{
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    float dst = 0.0;
    double a = 0;
    tf::StampedTransform transform_target;
    tf::TransformBroadcaster target_frame_broadcaster;
    tf::TransformListener target_camera_listener;
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        target_frame_broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(1.57, 1.57, 0.0), tf::Vector3(0.0, 0.0,    1.05)), ros::Time::now(),"tag_0", "target_point"));
        double roll, pitch, yaw, angle;
        try
        {
            //get the relative position (on rviz)
            target_camera_listener.lookupTransform("/camera_link", "/target_point", ros::Time(0), transform_target);
            
            tf::Matrix3x3(transform_target.getRotation()).getEulerYPR(yaw, pitch, roll);
            ROS_INFO("Actual EulerRPY : [%f, %f, %f]", roll, pitch, yaw);
            angle = yaw / M_PI * 180;
            dst = sqrt(pow(transform_target.getOrigin().x(), 2) + pow(transform_target.getOrigin().y(), 2));
            
            ROS_INFO("DST : %f ; ANGLE : %f", dst, angle);
            ROS_INFO("Acutal position : [ x =  %f, y = %f ]", transform_target.getOrigin().x(), transform_target.getOrigin().y());
            if (dst <=0.05 && angle <= 1)
            {
                vel_msg.linear.x = 0;   
                vel_msg.angular.z = 0;
            }
            else
            {
                vel_msg.linear.x =  1 * tanh(3.8 * dst) ;
                a = atan( (-1)*transform_target.getOrigin().x() /transform_target.getOrigin().y());
                if (a > 0)
                {
                    vel_msg.angular.z = ( (3 * (M_PI_2 - a) / M_PI * 180) + ((-1) * angle));
                    if (vel_msg.angular.z >= 1.2)
                    {
                        vel_msg.angular.z = 1.2;
                    }
                    
                }
                else
                {
                    vel_msg.angular.z = ( (3 * (-M_PI_2 - a) / M_PI * 180) + ((-1) * angle)) ; 
                    if (vel_msg.angular.z <= -1.2)
                    {
                        vel_msg.angular.z = -1.2;
                    }
                    
                }
            }
            ROS_INFO("alpha : %f ; angular : %f; linear : %f", a,
            vel_msg.angular.z,
            vel_msg.linear.x);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            vel_msg.linear.x = 0;
            vel_msg.angular.z =0;
        }
        // vel_msg.linear.x = 0;
        // vel_msg.angular.z =0;
        pub.publish(vel_msg);        
        rate.sleep();
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_tf_follower");
    ros::NodeHandle nh;
    
    method_two(nh);

    return 0;
}
