#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2)
{
  try
  {
    ROS_INFO("Image Recieved");
    cv::imshow("left", cv_bridge::toCvShare(msg1, "bgr8")->image);
    cv::imshow("right", cv_bridge::toCvShare(msg2, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg1->encoding.c_str());
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg2->encoding.c_str());
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "stereo_image_proc");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::SubscriberFilter left_sub, right_sub;
  image_transport::ImageTransport it(nh);
  left_sub.subscribe(it, "left/image", 1, image_transport::TransportHints("raw"));
  right_sub.subscribe(it, "right/image", 1, image_transport::TransportHints("raw"));

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

  boost::shared_ptr<ApproximateSync> approximate_sync;

  approximate_sync.reset(new ApproximateSync(ApproximatePolicy(1),left_sub,right_sub));
  approximate_sync->registerCallback(boost::bind(imageCallback, _1, _2));

  ros::spin();
  cv::destroyWindow("view");
}
