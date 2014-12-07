#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>

using namespace ar_track_alvar;
using namespace message_filters;
using namespace std;

ros::Publisher tag_pub;

void callback(const AlvarMarkersConstPtr& tags1, const AlvarMarkersConstPtr& tags2)
{

	AlvarMarkers tags;
	tags.header = tags1->header;
	//tags.markers = tags1->>markers;
	std::vector<int> ids_in_tags;
// cout << "Human ---- ";
	for(int i=0;i<tags1->markers.size();i++){
    if(tags1->markers[i].pose.pose.position.y > 1.0){
		  tags.markers.push_back(tags1->markers[i]);
      // cout << tags1->markers[i].id << ", ";
    }
	}
  // cout<<endl;
  // cout <<"Robot ---";
  //cout << "ids1 size " << tags.markers.size()<<endl; 

	for(int i=0;i<tags2->markers.size();i++){
    if(tags2->markers[i].pose.pose.position.y < 1.0)
		  tags.markers.push_back(tags2->markers[i]);
      // cout << tags2->markers[i].id << " ,";
	}
/*
	for(int i=0;i<tags2->>markers.size();i++){
		std::vector<int>::iterator it;
		it = std::find(ids_in_tags.begin(), ids_in_tags.end(), tags2->>markers[i].id);
		if (it == ids_in_tags.end()){
			tags.markers.push_back(tags2->>markers[i]);
		}
	}
	*/
  // cout << "ids1+2 size " << tags.markers.size()<<endl; 
	//ROS_INFO_STREAM(tags);
	
	tag_pub.publish(tags);
	

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_pose_marker_sync");

	ros::NodeHandle nh_;
	
	tag_pub = nh_.advertise<AlvarMarkers>("ar_pose_marker", 1);

	message_filters::Subscriber<AlvarMarkers> sub1(nh_, "ar_pose_marker1", 1);
	message_filters::Subscriber<AlvarMarkers> sub2(nh_, "ar_pose_marker2", 1);
	TimeSynchronizer<AlvarMarkers, AlvarMarkers> sync(sub1, sub2, 10);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	return 0;
}
