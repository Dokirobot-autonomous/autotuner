#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::Publisher pub;

void playRosbag(const std_msgs::String::ConstPtr& msg){
	std::string cmd;
	std::stringstream ss;
	std_msgs::Empty done;
	ss << "rosbag play " << msg->data << " -r 2.0"  ;
	cmd = ss.str();
	ROS_INFO("execute cmd\n");
	system(cmd.c_str());
	ROS_INFO("finish\n");
	pub.publish(done);
}
int main(int argc,char **argv){
	ros::init(argc, argv, "bagplayer");
	ros::NodeHandle n;
	pub = n.advertise<std_msgs::Empty>("done", 1000);
	ros::Subscriber sub = n.subscribe("play",1000,playRosbag);
	ROS_INFO("start\n");

	ros::spin();
	return 0;	
}
