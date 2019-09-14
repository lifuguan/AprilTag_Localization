#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
static const std::string IMAGE_TOPIC = "/my_robot/camera1/image_raw";
Mat frame;

vpDisplayX *d = NULL;
vpImage<unsigned char> I; // for gray images
vpHomogeneousMatrix cMo;
bool init = true;
vpDetectorQRCode detector;
// Camera parameters should be adapted to your camera
vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);

// 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
std::vector<vpPoint> point;

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo);

void imageCallback(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qrcode_detecter");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(IMAGE_TOPIC, 1, imageCallback);

  try
  {
    point.push_back(vpPoint(-0.06, -0.06,
                            0)); // QCcode point 0 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, -0.06,
                            0));             // QCcode point 1 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, 0.06, 0)); // QCcode point 2 3D coordinates in plane Z=0
    point.push_back(vpPoint(-0.06, 0.06,
                            0)); // QCcode point 3 3D coordinates in plane Z=0
  }
  catch (const vpException &e)
  {
    // std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
  ros::spin();
  return 0;
}
int time_ = 1;
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {

    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    // 第一次进入回调 ， 初始化现实界面
    if (time_ == 1)
    {
      vpImageConvert::convert(frame, I);
      d = new vpDisplayX();
      d->init(I);
      vpDisplay::setTitle(I, "visp_auto_tracker debug display");
      time_ += 1;
    }

    // 从opencv Mat格式转换到 VISP格式
    vpImageConvert::convert(frame, I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) // a click to exit
    {
    }

    bool status = detector.detect(I);

    std::ostringstream legend;
    legend << detector.getNbObjects() << " bar code detected";
    vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);

    if (status)
    { // true if at least one QRcode is detected
      for (size_t i = 0; i < detector.getNbObjects(); i++)
      {

        std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image

        // 信息读取
        vpRect bbox = detector.getBBox(i);
        vpDisplay::displayRectangle(I, bbox, vpColor::green);
        vpDisplay::displayText(I, (int)bbox.getTop() - 20, (int)bbox.getLeft(),
                               "Message: \"" + detector.getMessage(i) + "\"", vpColor::red);
        for (size_t j = 0; j < p.size(); j++)
        {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j] + vpImagePoint(10, 0), number.str(), vpColor::blue);
        }

        // 位姿读取
        for (size_t j = 0; j < p.size(); j++)
        {
          vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
          std::ostringstream number;
          number << j;
          vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
        }

        computePose(point, p, cam, init,
                    cMo); // resulting pose is available in cMo var
        std::cout << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                  << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl;
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
      }
    }
    vpDisplay::displayText(I, (int)I.getHeight() - 15, 10, "A click to quit...", vpColor::red);
    vpDisplay::flush(I);
    vpImageConvert::convert(I, frame);

    if (vpDisplay::getClick(I, false))
    {
    }

    vpTime::wait(40);
  }
  catch (cv_bridge::Exception &e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip, const vpCameraParameters &cam,
                 bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++)
  {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true)
  {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}