/*
 * btp_vision_node_cleaned.cpp
 * This node will publish the segmented parts of the image
 *  Created :Sep, 2015
 *      Author: Zubin
 */

#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <iostream>
#include <message_filters/subscriber.h>
//#include "image_segment.hpp"
#include "depth_and_rgb_transport.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");     //initialize this node with the name image_converter
  ImageConverter ic;


  ROS_INFO("Started converting the images using cv_bridge...");            
  ros::spin();
  return 0;
}