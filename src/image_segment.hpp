/*
* image_segment.hpp
* This is where the image processing and segmentation takes place 
*  Created :Sep, 2015
*      Author: Zubin
*/
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv2/opencv.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream> 
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;

void convertCameraInfoToMats(
  const CameraInfoConstPtr& camera_info_msg,
  cv::Mat& intr,
  cv::Mat& dist)
{
  // set intrinsic matrix from K vector
  intr = cv::Mat(3, 3, CV_64FC1);
  for (int idx = 0; idx < 9; ++idx)
  {
    int i = idx % 3;
    int j = idx / 3;
    intr.at<double>(j, i) = camera_info_msg->K[idx];
  }
  
  // set distortion matrix from D vector
  int d_size = camera_info_msg->D.size();
  dist = cv::Mat(1, d_size, CV_64FC1);
  for (int idx = 0; idx < d_size; ++idx)
  {
    dist.at<double>(0, idx) = camera_info_msg->D[idx];   
  }
}

// double get_xyz(int u,int v,const cv::Mat& intr_rect_ir,uint16_t z)
// {
//   double pt[] = {0,0,0};
//   double cx = intr_rect_ir.at<double>(0,2);
//   double cy = intr_rect_ir.at<double>(1,2);
//   double fx_inv = 1.0 / intr_rect_ir.at<double>(0,0);
//   double fy_inv = 1.0 / intr_rect_ir.at<double>(1,1);

//   double z_metric = z * 0.001;
             
//   pt[0] = z_metric * ((u - cx) * fx_inv);
//   pt[1] = z_metric * ((v - cy) * fy_inv);
//   pt[2] = z_metric;  
//   return pt[];   
// }


void img_seg_callback(const ImageConstPtr& image1, const ImageConstPtr& image2, const CameraInfoConstPtr& cam_info)
{

  // Solve all of perception here...
 //ROS_INFO("Time Sync is working...");
 //cout << "time sync is working" << endl;
 cv_bridge::CvImagePtr cv_ptr_color;
 try
 {
   cv_ptr_color = cv_bridge::toCvCopy(image2, sensor_msgs::image_encodings::BGR8);
 }
 catch (cv_bridge::Exception& e)
 {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
 }

 cv_bridge::CvImagePtr cv_ptr_depth;
 try
 {
   cv_ptr_depth = cv_bridge::toCvCopy(image1, sensor_msgs::image_encodings::TYPE_16UC1);
 }
 catch (cv_bridge::Exception& e)
 {
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
 }

 Mat depthOriginal = cv_ptr_depth->image; 
 Mat imgOriginal = cv_ptr_color->image;

 Mat imgHSV;
 Mat imgThresholded;
 Mat contour_drawn = Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC3);
 vector<vector<Point> > contours; // Vector for storing contour
 vector<Vec4i> hierarchy;
 int largest_area=0;
 int largest_contour_index=0;
 Rect bounding_rect;


 cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 inRange(imgHSV, Scalar(145, 82, 155), Scalar(179, 255, 255), imgThresholded); //Threshold the image
 //erode and dilate again to fill out the holes in the image
 erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
 dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
 dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
 erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

 imshow("Thresholded Image", imgThresholded); //show the thresholded image
 findContours(imgThresholded, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
 
 //find the largest contour and the corresponding bounding rectangle for it
 for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
 {
    double a=contourArea( contours[i],false);  //  Find the area of contour
    if(a>largest_area)
    {
     largest_area=a;
     largest_contour_index=i;                //Store the index of largest contour
     bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
   }
 }

 //Draw a rectangle around the largest contour
 rectangle(imgOriginal, bounding_rect,  Scalar(0,255,0),10, 8,0);
 drawContours(contour_drawn, contours, largest_contour_index, Scalar(0,255,0), CV_FILLED, 8);
 //Show the different images for debugging purposes
 imshow("contours drawn",contour_drawn);
 imshow("Original", imgOriginal);
 imshow("Depth", depthOriginal);
 
 //imshow(OPENCV_WINDOW, cv_ptr->image);
 waitKey(3);
 

 //check whether a contour is detected to avoid segmentation fault
 if (!contours.empty()) 
 {

   int nRows = depthOriginal.rows;    //number of rows in depth image (resolution is 640x480)
   int nCols = depthOriginal.cols;    //number of columns in depth image

   int x,y;
   int point_counter = 0;             //counter for number of valid points inside detected blob
   float distance_sum = 0;
   double xyz_sum[] = {0,0,0};
   float distance_of_blob = 0;        
   unsigned short distance_of_pixel;
  cv::Mat intrinsic_mat, distortion_mat;
  convertCameraInfoToMats(cam_info, intrinsic_mat, distortion_mat);
  double cx = intrinsic_mat.at<double>(0,2);
  double cy = intrinsic_mat.at<double>(1,2);
  double fx_inv = 1.0 / intrinsic_mat.at<double>(0,0);
  double fy_inv = 1.0 / intrinsic_mat.at<double>(1,1);

   for( y = 0; y < nRows; y++)
   {

      
      for ( x = 0; x < nCols; x++)
      {
        //pointPolygonTest returns 1 if the Pont(x,y) is inside the contour, -1 if outside, 0 if on the contour
        if(pointPolygonTest(contours[largest_contour_index],Point(x,y),false) == 1)
       {
        distance_of_pixel = depthOriginal.at<unsigned short>(Point(x,y));   
        
        //if uint16 distance == 0 then it means that the point is not detected by depth camera
        if(distance_of_pixel != 0)
        {
         //distance_sum += float(distance_of_pixel);
        	
  			double pixel_xyz[] = {0,0,0};
  			double z_metric = distance_of_pixel * 0.1;
  			
             
  			pixel_xyz[0] = z_metric * ((x - cx) * fx_inv);
 			pixel_xyz[1] = z_metric * ((y - cy) * fy_inv);
  			pixel_xyz[2] = z_metric;  
  			xyz_sum[0] += 	pixel_xyz[0];
			xyz_sum[1] += 	pixel_xyz[1];
			xyz_sum[2] += 	pixel_xyz[2];


         point_counter++;


        }
        
       }

       }
    }

    if(point_counter != 0) {ROS_INFO_STREAM("X Y and Z position of blob in cms is" <<(xyz_sum[0]/double(point_counter)) << ", " <<(xyz_sum[1]/double(point_counter)) << ", " << (xyz_sum[2]/double(point_counter)) << endl);}
    
    //following is the case when rgb detects the blob, but it is out of the distance range of depth camera
    else ROS_INFO_STREAM("Blob is detected but not in range" << endl);
  }
  //if no countour is detected
  else ROS_INFO_STREAM("Blob is not detected" << endl);
}


