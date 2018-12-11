#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

int main(int argc, char* argv[])
{
  //Check for command line arguments
  if(argv[1] == NULL) return 1;

  //Create node named image_publisher
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;							//node address handle
  image_transport::ImageTransport it(nh);				//image transport pipe
  image_transport::Publisher pub=it.advertise("camera/image", 1);	//publisher on topic camera/image

  //Conversion of argv[1] to integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;

  //Is it a number?
  if(!(video_sourceCmd >> video_source)) return 1;

  //Capture video frames with OpenCV
  cv::VideoCapture cap(video_source);
  //Can open video source?
  if(!cap.isOpened()) return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while (nh.ok())
  {
    cap >> frame;
    //Frame is full of content?
    if(!frame.empty())
    {
      ROS_INFO("Publishing image");
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
return 0;
}
//End
