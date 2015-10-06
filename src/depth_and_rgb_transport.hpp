/*
 * depth_and_rgb_transport.hpp
 * This is where the image in converted from ros format to opencv format
 *  Created :Sep, 2015
 *      Author: Zubin
*/

using namespace cv;
using namespace std;
// using namespace sensor_msgs;
// using namespace message_filters;

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_color;
  image_transport::Subscriber image_sub_depth;
  image_transport::Publisher image_pub_color;
  image_transport::Publisher image_pub_depth;
  
 
public:
  ImageConverter()
    : it_(nh)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_depth = it_.subscribe("/camera/depth_registered/image_raw", 1,&ImageConverter::pub_converted_depth_callback, this);
    image_sub_color = it_.subscribe("/camera/rgb/image_rect_color", 1,&ImageConverter::pub_converted_color_callback, this);
    image_pub_depth = it_.advertise("/image_converter/output_depth", 1);
    image_pub_color = it_.advertise("/image_converter/output_color", 1);
    
    
  }


   //This is the callback function for the depth image subscriber. This piece of code is executed every time an image
   //is recieved from the /camera/depth_registered/image_raw topic
   void pub_converted_depth_callback(const sensor_msgs::ImageConstPtr& msg)
  {
     cv_bridge::CvImagePtr cv_ptr_depth;
     try
     {
       cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }

   image_pub_depth.publish(cv_ptr_depth->toImageMsg());
     }

  //this callback function will publish the color  images
  void pub_converted_color_callback(const sensor_msgs::ImageConstPtr& msg)
  {
     cv_bridge::CvImagePtr cv_ptr_color;
     try
     {
       cv_ptr_color = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }

    image_pub_color.publish(cv_ptr_color->toImageMsg());
    
  }


};