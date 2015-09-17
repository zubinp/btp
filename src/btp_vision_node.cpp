#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include"opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
//    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
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

    Mat imgOriginal = cv_ptr->image;


  Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(58, 39, 95), Scalar(94, 255, 255), imgThresholded); //Threshold the image
      
  // //morphological opening (remove small objects from the foreground)
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  // //morphological closing (fill small holes in the foreground)
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  imshow("Thresholded Image", imgThresholded); //show the thresholded image
   vector<vector<Point> > contours; // Vector for storing contour
  vector<Vec4i> hierarchy;
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;

  findContours(imgThresholded, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
 
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

rectangle(imgOriginal, bounding_rect,  Scalar(0,255,0),10, 8,0);

  imshow("Original", imgOriginal);


    //imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
    
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}