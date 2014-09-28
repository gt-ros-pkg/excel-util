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
    
    sub = it.subscribe("/scan_barcode_cam/image_raw", 1, &BarcodeScan::imageCallback, this);
    
    process_im_ = false; // process images 
    currently_process_ = false; // currently processing an image?
  }

  bool find_tag(string tag_name, float timeout=2.);

private:

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  image_transport::ImageTransport it;
  image_transport::Subscriber sub;
  ros::NodeHandle nh_; 
  bool process_im_, currently_process_;
  cv_bridge::CvImagePtr cv_ptr;

};

bool BarcodeScan::find_tag(string tag_name, float timeout/*=2.*/)
{
  process_im_ = true;
  bool found_tag = false;
  int frames = 0;
  int total_frames = floor(timeout * SCAN_FRAME_RATE);
  
  while(!(frames>total_frames && found_tag)){
    if (currently_process_){
      frames++;
      DmtxImage      *img;
      DmtxDecode     *dec;
      DmtxRegion     *reg;
      DmtxMessage    *dmtx_msg;
    
      img = dmtxImageCreate(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
			    DmtxPack8bppK);
      assert(img != NULL);
      ROS_INFO("A %d %d", img->width, img->height);

      dec = dmtxDecodeCreate(img, 1);
      assert(dec != NULL);
      ROS_INFO("B");

      reg = dmtxRegionFindNext(dec, NULL);
    
      while(reg!=NULL){
	ROS_INFO("C");
	dmtx_msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
	ROS_INFO("D");
	if(dmtx_msg != NULL) {
	  std::string str_dmtx((char*)reinterpret_cast<char*>(dmtx_msg->output));
	  if(str_dmtx == tag_name){
	    found_tag = true;
	    ROS_INFO("E");
	    const char* output = reinterpret_cast<const char*>(dmtx_msg->output);
	    ROS_INFO("Message: %s", output);
	    dmtxMessageDestroy(&dmtx_msg);
	    
	    //debug
	    cout << "Message Found:" << endl;
	    break;
	  }
	} else {
	  ROS_INFO("Message not found");
	}
	dmtxRegionDestroy(&reg);
      }

      ROS_INFO("F");
      dmtxDecodeDestroy(&dec);
      dmtxImageDestroy(&img);
      ROS_INFO("G");
    
      currently_process_ = false;
    }
  }

  process_im_ = false;
  return found_tag;
}

void BarcodeScan::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  if(process_im_ && !currently_process_){

    ROS_INFO("IMG");
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    currently_process_ = true;

  }
  return;
}

#endif
