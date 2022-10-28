#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <tf/tf.h>
#include <math.h>


// ROS Image Topic name : iris/down_camera_link/down_raw_image

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

geometry_msgs::PoseStamped pose;
double rad = 0;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber pos_sub_;
  ros::Publisher target_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iris/flow_camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/iris/flow_camera/image_raw/cv_output", 1);

    pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10, &ImageConverter::pose_cb, this);
    target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_position", 10);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose = *msg;

    tf::Quaternion q(
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    rad = yaw;
    
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

    cvtColor(cv_ptr->image, cv_ptr->image, COLOR_BGR2GRAY);
    medianBlur(cv_ptr->image, cv_ptr->image, 5);

	  vector<Vec3f> circles;
  	HoughCircles(cv_ptr->image, circles, HOUGH_GRADIENT, 1, 100, 50, 35, 0, 0);

    // int row_size = cv_ptr->image.rows;
    // int col_size = cv_ptr->image.cols;
    // ROS_INFO("image row : %d , image col : %d", row_size, col_size);
	
    //Draw Circles on the image describe the Circle Shape
	  for (size_t i = 0; i < circles.size(); i++)
  	{
  		Vec3i c = circles[i];
	  	Point center(c[0], c[1]);
  		int radius = c[2];

      int origin_row = (cv_ptr->image.rows - 1) / 2;
      int origin_col = (cv_ptr->image.cols - 1) / 2;

		  circle(cv_ptr->image, center, radius, Scalar(0, 255, 0), 2);
		  circle(cv_ptr->image, center, 2, Scalar(0, 0, 255), 3);

      float error_col = 0.05 * (c[0] - origin_col)/(origin_col);
      float error_row = 0.05 * (c[1] - origin_row)/(origin_row); //x ->col
      

      ROS_INFO("center row : %d / row difference : %f", c[1], error_row);
      ROS_INFO("center col :%d / col difference : %f", c[0], error_col);
      
      if((fabs(rad) > (M_PI / 2) && fabs(rad) < M_PI) || (-1*rad > 0 && -1*rad < (M_PI/2))){
        pose.pose.position.x += error_col * sin(M_PI - rad);
      }
      else
      pose.pose.position.x -= error_col * sin(M_PI - rad);
      
      if(fabs(rad) > (M_PI * 1.5) && fabs(rad) < (2 * M_PI))
      pose.pose.position.y -= error_col * cos(M_PI - rad);
      else
      pose.pose.position.y += error_col * cos(M_PI - rad);

      // pose.pose.position.z += error_row;
      target_pub_.publish(pose);
	  }


	  imshow(OPENCV_WINDOW, cv_ptr->image);
	  waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());


  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  ros::spin();
  return 0;
}