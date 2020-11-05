#include "Odometry.h"
#include "Pose.h"
#include "csv.h"
#include <tf/transform_listener.h>
static void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void Odometry_C::load(char *filename,FILETYPE type){
	uint64_t time;	
	double roll,pitch,yaw;
	std::string str;
	Eigen::Vector3d pos;
	geometry_msgs::Quaternion orientation;

	io::CSVReader<8> file(filename);
	if(type == TYPE_ODOMETRY){
	file.read_header(io::ignore_extra_column,
			"field.header.stamp",
			"field.pose.pose.position.x",
			"field.pose.pose.position.y",
			"field.pose.pose.position.z",
			"field.pose.pose.orientation.x",
			"field.pose.pose.orientation.y",
			"field.pose.pose.orientation.z",
			"field.pose.pose.orientation.w"
			);
	}
	else if(type == TYPE_POSE){

	file.read_header(io::ignore_extra_column,
			"field.header.stamp",
			"field.pose.position.x",
			"field.pose.position.y",
			"field.pose.position.z",
			"field.pose.orientation.x",
			"field.pose.orientation.y",
			"field.pose.orientation.z",
			"field.pose.orientation.w"
			);
	}
	else{
		std::cout << "file type error" << std::endl;
	}
	while(file.read_row(
				str,
				pos.x(),	
				pos.y(),
				pos.z(),
				orientation.x,
				orientation.y,	
				orientation.z,
				orientation.w)
			){

		GetRPY(orientation,roll,pitch,yaw);
		time = std::stoull(str); 
		pose.emplace_back(time,pos,roll,pitch,yaw);
	}
}

void Odometry_C::print(bool flag){
	std::cout << "odom size:" << pose.size() << std::endl;
	if(flag){
		for(int i=0;i<pose.size();i++){
			pose[i].print();
		}
	}
}
void Odometry_C::insert(const Pose_C &in){
	pose.push_back(in);
}

void Odometry_C::extract(const Odometry_C &ref){
	Odometry_C out;	
	int ref_idx;
	int target_idx;
	
	uint64_t ref_stamp,target_stamp;
	uint64_t delta,min;
	const int timeout = 100000;
	target_idx = 0;
	for(ref_idx=0;ref_idx<ref.pose.size();ref_idx++){
		min = 0xFFFFFFFFFFFFFFFF;
		ref_stamp = ref.pose[ref_idx].getTime();
		for(target_idx=0;target_idx<pose.size();target_idx++){
			target_stamp = this->pose[target_idx].getTime();
			if(target_stamp > ref_stamp) delta=target_stamp-ref_stamp;
			else delta = ref_stamp-target_stamp;
			if(delta<=min) min = delta;
			else {
				out.insert(this->pose[target_idx-1]);
				break;
			}
		}

	}
	this->pose = out.pose;	
}
