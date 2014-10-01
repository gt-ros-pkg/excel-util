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
  
  while(!(frames>=total_frames )){
    if (currently_process_){
      cout << "TIME to get in " << t_get_in.elapsed() << endl;

      cout << "Processing Frame" << endl;
      frames++;
      DmtxImage      *img;
      DmtxDecode     *dec;
      DmtxRegion     *reg;
      DmtxMessage    *dmtx_msg;
    
      img = dmtxImageCreate(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
			    DmtxPack8bppK);
      assert(img != NULL);
      // ROS_INFO("A %d %d", img->width, img->height);

      dec = dmtxDecodeCreate(img, 1);
      assert(dec != NULL);
      // ROS_INFO("B");

      reg = dmtxRegionFindNext(dec, NULL);
    
      while(reg!=NULL){
	// ROS_INFO("C");
	dmtx_msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
	// ROS_INFO("D");
	if(dmtx_msg != NULL) {
	  std::string str_dmtx((char*)reinterpret_cast<char*>(dmtx_msg->output));
	  ROS_INFO("%s tag found", str_dmtx.c_str());
	  
	  for(int i=0;i<tag_names.size();i++){
	    if(compare_tags(str_dmtx, tag_names[i])){
	      found_tags[i] = true;
	      // ROS_INFO("E");
	      const char* output = reinterpret_cast<const char*>(dmtx_msg->output);
	      ROS_INFO("Message: %s", output);
	      dmtxMessageDestroy(&dmtx_msg);
	    
	      //debug
	      cout << "Message Found:" << endl;
	      break;
	    }
	  }
	} else {
	  // ROS_INFO("Message not found");
	}
	dmtxRegionDestroy(&reg);
	reg = dmtxRegionFindNext(dec, NULL);
      }

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
    if (t1c[i] == t2c[i]){diffs++;}
  }
  return (diffs<=num_diffs_allowed);
    }
#endif
