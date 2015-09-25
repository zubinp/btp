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
#include "image_segment.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");     //initialize this node with the name image_converter
  ImageConverter ic;            
  ros::spin();
  return 0;
}