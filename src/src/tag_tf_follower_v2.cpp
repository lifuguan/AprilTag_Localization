#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

Mat source_image;
geometry_msgs::Pose pose_msg;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        source_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        line(source_image, Point(0, 400), Point(800, 400), Scalar(0, 155, 155), 2);
        line(source_image, Point(400, 0), Point(400, 800), Scalar(0, 155, 155), 2);
        putText(source_image, "position : " + std::to_string((double)pose_msg.position.x) + ", " + std::to_string((double)pose_msg.position.y), Point(10,20),
           FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2);
        imshow("ResultView", source_image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void poseCallBack(const geometry_msgs::Pose pose)
{
    pose_msg = pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_tf_follower_v2");
    ros::NodeHandle nh;
    namedWindow("ResultView");
    startWindowThread();
    pose_msg.position.x = 0; pose_msg.position.y = 0;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/tag_detections_image", 1, imageCallback);
    ros::Subscriber pose_sub = nh.subscribe("/pose", 1, poseCallBack);
    ros::spin();

    destroyWindow("ResultView");
    return 0;
}
