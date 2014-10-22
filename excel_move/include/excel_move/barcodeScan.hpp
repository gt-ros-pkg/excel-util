#ifndef BARCODE_SCAN
#define BARCODE_SCAN
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dmtx.h>
#include <iostream>
#include <boost/timer.hpp>

#define SCAN_FRAME_RATE 7

/**
   Class to scan for bins. 
**/

using namespace std;

class BarcodeScan
{
public:
  BarcodeScan(ros::NodeHandle& nh): nh_(nh), it(nh)
  {
    cout << "Start constructor" << endl;

    sub = it.subscribe("/scan_barcode_cam/image_raw", 1, &BarcodeScan::imageCallback, this);
    
    process_im_ = false; // process images 
    currently_process_ = false; // currently processing an image?

    deviation_ = 60.0;
    timeout_ = 1.0;
    threshold_ = 180.0;
    min_edge_ = 5.0;
    max_edge_ = 100.0;
    processing_option_ = 1;
    nh_.getParam("deviation", deviation_);
    nh_.getParam("timeout", timeout_);
    nh_.getParam("min_edge", min_edge_);
    nh_.getParam("max_edge", max_edge_);
    nh_.getParam("processing_option", processing_option_);
    nh_.getParam("threshold", threshold_);

    ROS_INFO("PARAMS: %f %f %f %f %d %f", deviation_, timeout_, min_edge_, max_edge_, processing_option_, threshold_);
    cv::namedWindow("Image Display");
    cvStartWindowThread();
    cout << "End constructor" << endl;
  }

  vector<bool> find_tag(vector<string> tag_names);
  bool compare_tags(string t1, string t2);
private:

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  
  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  ros::NodeHandle nh_; 
  bool process_im_, currently_process_;
  cv_bridge::CvImagePtr cv_ptr;

  double deviation_, timeout_, min_edge_, max_edge_, threshold_;
  int processing_option_;
};

vector<bool> BarcodeScan::find_tag(vector<string> tag_names)
{
  boost::timer t_get_in, t_total;

  process_im_ = true;
  vector<bool> found_tags;
  for(int i=0;i<tag_names.size();i++){
    found_tags.push_back(false);
  }

  int frames = 0;
  int total_frames = 1;//floor(timeout * SCAN_FRAME_RATE);
  cv::Mat image, image_thresh, mask;
  cv::Mat di = cv::Mat(30,30,CV_8UC1);
  cv::Mat er = cv::Mat(5,5,CV_8UC1);
  while(!(frames>=total_frames )){
   
    if (currently_process_){
      cout << "TIME to get in " << t_get_in.elapsed() << endl;

      cout << "Processing Frame" << endl;
      frames++;
      DmtxImage      *img;
      DmtxDecode     *dec;
      DmtxTime      dmtx_timeout;
      DmtxRegion     *reg;
      DmtxMessage    *dmtx_msg;
      
      
      switch(processing_option_){
                case 1:
		cv_ptr->image.copyTo(image_thresh);
		break;

		case 2:
		cv::threshold(cv_ptr->image, image_thresh, threshold_, 255, cv::THRESH_TOZERO);
		break;

		case 3:
		cv::threshold(cv_ptr->image, image_thresh, threshold_, 255, cv::THRESH_BINARY);
		break;

		case 4:
		cv::threshold(cv_ptr->image, image_thresh, threshold_, 255, cv::THRESH_TOZERO );
		cv::floodFill(image_thresh, cv::Point(150,150), cv::Scalar(255.0));
		break;

		case 5:
		cv::threshold(cv_ptr->image, image_thresh, threshold_, 255, cv::THRESH_BINARY );
		cv::floodFill(image_thresh, cv::Point(150,150), cv::Scalar(255.0));
                //cv::dilate(image_thresh, image_thresh, cv::Mat(2,2,CV_8UC1));
		break;

		case 6:
		cv::threshold(cv_ptr->image, mask, threshold_, 255, cv::THRESH_BINARY );
		cv::erode(mask,mask,er); 
		cv::dilate(mask,mask,di); 
		cv::dilate(mask,mask,cv::Mat(5,5,CV_8UC1)); 
		cv::normalize(mask, mask, 0, 1, cv::NORM_MINMAX, CV_8UC1);
		cv::multiply(cv_ptr->image, mask,image_thresh);
		break;

		}
      
      cv::imshow("Image Display", image_thresh);
      // cv::imwrite("/home/asif/ur10_new_ws/src/dmtx_barcode_scan/data/image_thresh1.jpg",image_thresh);
      img = dmtxImageCreate(image_thresh.data, image_thresh.cols, image_thresh.rows, 
			    DmtxPack8bppK);
      assert(img != NULL);
      // ROS_INFO("A %d %d", img->width, img->height);

      dec = dmtxDecodeCreate(img, 1);
      assert(dec != NULL);
      // ROS_INFO("B");
      dmtxDecodeSetProp(dec, DmtxPropSquareDevn, deviation_);
      dmtxDecodeSetProp(dec, DmtxPropEdgeMin, min_edge_);
      dmtxDecodeSetProp(dec, DmtxPropEdgeMax, max_edge_);
      
      int count = 0;
     
      double t_old = t_total.elapsed();
      dmtx_timeout = dmtxTimeAdd(dmtxTimeNow(), timeout_);
      reg = dmtxRegionFindNext(dec, &dmtx_timeout);
      double t_new = t_total.elapsed();
      double t_tag = t_new - t_old;
      cout << "TIME to find next tag: " << t_tag<< endl;
      
      while(reg!=NULL){
	// ROS_INFO("C");
	dmtx_msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
	count++;
	cout<<"Function call Number: "<<count<<endl;
	// ROS_INFO("D");
	if(dmtx_msg != NULL) {
	  std::string str_dmtx((char*)reinterpret_cast<char*>(dmtx_msg->output));
	  ROS_INFO("%s tag found", str_dmtx.c_str());
	  
	  for(int i=0;i<tag_names.size();i++){
	    if(compare_tags(str_dmtx, tag_names[i])){
	      ROS_INFO("Tag Matched!");
	      found_tags[i] = true;
	      // ROS_INFO("E");
	      const char* output = reinterpret_cast<const char*>(dmtx_msg->output);
	      ROS_INFO("Message: %s", output);
	      dmtxMessageDestroy(&dmtx_msg);
	    
	      //debug
	      
	      break;
	    }
	  }
	} else {
	   ROS_INFO("Message not found");
	}
	
	dmtxRegionDestroy(&reg);
	
	
	t_old = t_total.elapsed();
	dmtx_timeout = dmtxTimeAdd(dmtxTimeNow(), timeout_);
	reg = dmtxRegionFindNext(dec, &dmtx_timeout);
	t_new = t_total.elapsed();
	t_tag = t_new - t_old;
	cout << "TIME to find next tag: " << t_tag<< endl;
      }
      cout<<"Outside While loop"<<endl;
      // ROS_INFO("F");
      dmtxDecodeDestroy(&dec);
      dmtxImageDestroy(&img);
      // ROS_INFO("G");
    
      currently_process_ = false;
    }
  }

  process_im_ = false;
  cout << "TIME total " << t_total.elapsed() << endl;
  return found_tags;
}

void BarcodeScan::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if(process_im_ && !currently_process_){
    cout << "Get to callback?" << endl;

    ROS_INFO("IMG");
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    currently_process_ = true;

  }
  return;
}

bool BarcodeScan::compare_tags(string t1, string t2)
{
  int num_diffs_allowed=1; int diffs=0;
  const char* t1c = t1.c_str();
  const char* t2c = t2.c_str();
  
  for(int i=0; i<t1.size(); ++i){
    if (t1c[i] != t2c[i]){diffs++;}
  }
  
  if(t1.size()!=t2.size())
  diffs = num_diffs_allowed+1;
  
  return (diffs<=num_diffs_allowed);
}

#endif
