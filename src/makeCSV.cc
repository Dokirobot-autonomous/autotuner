#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <autotuner/save.h>

std::vector<std::string>data;
const int MAX_ROW= 11;

void writeCSV(const char *filename){
	int row=1;
	std::ofstream file(filename);
	
	for(int i=0;i<data.size();i++){
		file << data[i];
		if(row==MAX_ROW){
			file << '\n';
			row = 0;
		}
		else file << ',';
		
		row++;
	}
	
	file.close();
}

bool saveCSV(autotuner::save::Request &req,
autotuner::save::Response &res){

	writeCSV(req.filename.c_str());
	res.success = true;
	return true;

}
void callback(const geometry_msgs::PoseStampedConstPtr &topic){
	ROS_INFO("call back\n");
	char str[30];
	sprintf(str,"%lu%09lu",topic->header.stamp.sec,topic->header.stamp.nsec);
	
	data.push_back(str);
	data.push_back(std::to_string(topic->header.seq));
	data.push_back(str);
	data.push_back(topic->header.frame_id);
	data.push_back(std::to_string(topic->pose.position.x));
	data.push_back(std::to_string(topic->pose.position.y));
	data.push_back(std::to_string(topic->pose.position.z));
	data.push_back(std::to_string(topic->pose.orientation.x));
	data.push_back(std::to_string(topic->pose.orientation.y));
	data.push_back(std::to_string(topic->pose.orientation.z));
	data.push_back(std::to_string(topic->pose.orientation.w));
	
}

void insert_label(void){
	data.push_back("%time");
	data.push_back("field.header.seq");
	data.push_back("field.header.stamp");
	data.push_back("field.header.frame_id");
	data.push_back("field.pose.position.x");
	data.push_back("field.pose.position.y");
	data.push_back("field.pose.position.z");
	data.push_back("field.orientation.x");
	data.push_back("field.orientation.y");
	data.push_back("field.orientation.z");
	data.push_back("field.orientation.w");
}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/orb_slam2_stereo/lidar_pose",1000,callback);
	insert_label();
	ros::ServiceServer service = n.advertiseService("saveCSV",saveCSV);
	ROS_INFO("start\n");
	system("sleep 10");

	ros::spin();
	return 0;
}
