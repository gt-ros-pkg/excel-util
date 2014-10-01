
#include <ros/ros.h>
#include <excel_move/scanning.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scanning");
	ros::NodeHandle nh;

	usleep(1000*1000);
	
	Scanning scanning(nh);

	vector<string> good_tags;
	good_tags.push_back("333");
	good_tags.push_back("555");
	good_tags.push_back("999");

	vector<string> bad_tags;
	bad_tags.push_back("111");
	bad_tags.push_back("777");
	//bad_tags.push_back("999");

	cout << scanning.scan_it(good_tags, bad_tags);
	
	ros::shutdown();
	return 0;
}
