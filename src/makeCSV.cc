#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <autotuner/save.h>

//global variables
std::vector<std::string>data;
std::string csv_filename;
const int MAX_ROW= 11;
bool play_flag = false;

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

bool saveCSV(const std::string &filename){
	writeCSV(csv_filename.c_str());
	return true;
}
void getDoneSignal(const std_msgs::Empty &msg){
	printf("finsh rosbag\n");
	saveCSV("");
	data.clear();
	play_flag = false;	

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
	std::stringstream ss;
	std::string basefilename("orb_odom_");
	std::string bagfilename(argv[1]);
	std_msgs::String msg;
	int file_count = 0;
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;
	ros::Subscriber sub_pose = n.subscribe("/orb_slam2_stereo/lidar_pose",1000,callback);
	ros::Subscriber sub_done = n.subscribe("/done",1000,getDoneSignal);
	ros::Publisher pub_play_signal = n.advertise<std_msgs::String>("/play",1000);
	insert_label();
	ros::ServiceServer service = n.advertiseService("saveCSV",saveCSV);
	ROS_INFO("start\n");
	ros::Rate loop_rate(10);

	sleep(5);

//main loop
	while(ros::ok()){
		
		if(!play_flag){
			//csvファイル名を設定
			ss << basefilename << file_count << ".csv";
			csv_filename = ss.str();
			ROS_INFO("play bag\n");	
			//bagファイル名を指定
			msg.data = bagfilename;
			play_flag = true;
			pub_play_signal.publish(msg);
			file_count++;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
