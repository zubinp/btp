/*
 * image_sync.hpp
 * This node takes the  depth image and color image, time syncs it and then sends these synced images for image segmentation 
 *  Created :Sep, 2015
 *      Author: Zubin
*/

#include "image_segment.hpp"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sync_and_segment");

  ros::NodeHandle nh;
  message_filters::Subscriber<Image> image1_sub(nh, "/image_converter/output_depth", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/image_converter/output_color", 1);
   
  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&img_seg_callback, _1, _2));

  ros::spin();

  return 0;
}





