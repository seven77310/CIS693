#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <geometry_msgs/Twist.h>

static const std::string RGB_WINDOW = "Image window";
static const std::string DEPTH_WINDOW = "Depth window";

class ImageConverter
{ 
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_ , image_depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist base_cmd;
  float cur_dis;
  float pre_dis;
  int min_dis;
  int row;
  int col, center_col,diff;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);
    image_depth_sub_ = it_.subscribe("/camera/depth/image", 10, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    min_dis = 1.0;
    cv::namedWindow(RGB_WINDOW);
    cv::namedWindow(DEPTH_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    cv::destroyWindow(DEPTH_WINDOW);
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

    row = 0; 
    col = 0;
 
    // Now we want to find the center pixel point of the ball
    // To do that we define a threshold to be compared with all image pixels and find the ball pixels
    int threshold = 50;
    // In the next step, we want to find the maximum value and minimum value for ball's row and column pixels 
    int min_x,min_y,max_x,max_y;
    max_x=max_y=0;
    min_y=min_x=1000;
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
	  // get red, green and blue vlues for each image pixel
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
	    // find the euclidean distance of image pixel's r,g and b with orange color r,g and b (255,188,47)
            int dis = pow((r-255)*(r-255) + (g-188)*(g-188) + (b-47)*(b-47), 0.5); 
	    // compare each pixel's distance with threshold, to determine if it's orange or not
	    //If the distance was less than threshold, it means the pixel can be considered as ball pixel
            if (dis < threshold) {
	      //Now find the max and min value for row and column of ball 
		if (max_x<j){
			max_x=j;		
		}
		if (min_x>j){
			min_x=j;		
		}
		if(max_y<i){
			max_y=i;		
		}
		if(min_y>i){
			min_y=i;
		} 
            }
        }
    }
    // find the average value to set the center point 
    col=(max_x+min_x)/2;
    row=(max_y+min_y)/2;	

    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(RGB_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
        void depthCb(const sensor_msgs::ImageConstPtr& msg)
        {
                cv_bridge::CvImageConstPtr cv_ptr;
                try
                {
		  //Find center column of each image frame
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		  center_col = (cv_ptr->image.cols / 2) + 1;
		}
                catch (const cv_bridge::Exception& e)
                {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                }
		//Find the distance between the robot position and ball center-point
		cur_dis = cv_ptr->image.at<float>(row,col);
		//Find the distance between center column of ball and center column of image frame
		diff = center_col - col;
		// If the distance was more than 70 pixels,
		// it means the ball is in the left hand side of robot
		if( diff > 70) {
		  //then we want robot turn to left with a angular velocity 0.4
		base_cmd.angular.z = 0.4;

}
		// If the distance was less than 70 pixels,
		// it means the ball is in the right hand side of robot
		else if (diff<-70){
		  //so we want robot turn to right with a angular velocity 0.4
		base_cmd.angular.z = -0.4;
}
		//otherwise, robot is almost facing the ball
		else{
		  //and no need to turn in any direction
		base_cmd.angular.z = 0.0;
}
		if(cur_dis > 0)
		{	
		        // If the distance between the robot position and ball center-point was more than 1 meter
			if(cur_dis > min_dis + 0.1)
			{
			  //then go toward it with the linear velocity of 0.3
			    base_cmd.linear.x = 0.3;

			}
			// If the distance between the robot position and ball center-point was less than 1 meter
			else if (cur_dis < min_dis -0.1)
			{
			  //then go backward with the linear velocity of 0.3
			    base_cmd.linear.x = -0.3;
			}
			else 
			{
			  // If the distance between the robot position and ball center-point was almost 1 meter
			  //no action is needed
		 	base_cmd.linear.x= 0.0;
			}
		}
		 cmd_vel_pub_.publish(base_cmd);
		 cv::imshow(DEPTH_WINDOW, cv_ptr->image);
        }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
