#include <cstdlib>
#include <string>
#include <ros/ros.h>
int main(int argc,char **argv){
	std::string filename;
	ros::init(argc, argv, "bagplayer");
	ros::NodeHandle n;
	ros::spin();
	return 0;	
}
