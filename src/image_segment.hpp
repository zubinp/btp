/*
 * image_segment.hpp
 * This is where the image processing and segmentation takes place 
 *  Created :Sep, 2015
 *      Author: Zubin
*/

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh)
  {

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,&ImageConverter::img_seg_callback, this);
    namedWindow("Image window");
  }

  ~ImageConverter()
  {
    destroyWindow("Image window");
  }

   //This is the callback function for the image subscriber. This piece of code is executed every time an image
   //is recieved from the /camera/rgb/image_rect_color topic
   void img_seg_callback(const sensor_msgs::ImageConstPtr& msg)
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
   Mat imgThresholded;
   Mat contour_drawn = Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC3);
   vector<vector<Point> > contours; // Vector for storing contour
   vector<Vec4i> hierarchy;
   int largest_area=0;
   int largest_contour_index=0;
   Rect bounding_rect;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
   inRange(imgHSV, Scalar(58, 39, 95), Scalar(94, 255, 255), imgThresholded); //Threshold the image
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
   dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
   erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

   imshow("Thresholded Image", imgThresholded); //show the thresholded image
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
   drawContours(contour_drawn, contours, largest_contour_index, Scalar(0,255,0), CV_FILLED, 8);
   imshow("contours drawn",contour_drawn);
   imshow("Original", imgOriginal);


    //imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
    
    
  }
};