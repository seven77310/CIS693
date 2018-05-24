#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{ 
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_ , depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher base_cmd_pub_;
  geometry_msgs::Twist base_cmd;
  int cur_row, cur_col;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);

    depth_sub_ = it_.subscribe("/camera/depth/image", 10, &ImageConverter::depthCb, this);
    base_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    
    cur_row = 0;
    cur_col = 0;
  
    cv::namedWindow(OPENCV_WINDOW);
    
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    int dis_min = 1000000; 
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
            int dis = (r-255)*(r-255) + g*g + b*b; 
            if (dis < dis_min) {
                dis_min = dis; 
                cur_row = i; 
                cur_col = j; 
            }
        }
    }

    cv::circle(cv_ptr->image, cv::Point(cur_col, cur_row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
}


  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    float depth;
    
    try
    {
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
		
		depth = cv_ptr->image.at<float>(cur_row,cur_col);
    if (depth<(1.0-0.2) | depth>(1.0+0.2))
    {
      base_cmd.linear.x = depth - default_dist;
      base_cmd_pub_.publish(base_cmd);
    }
    
  }

 };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
