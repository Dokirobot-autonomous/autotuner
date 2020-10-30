#include "Odometry.h"
#include "Pose.h"
#include "csv.h"
static void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
void Odometry_C::insert(Pose_C &in){
	odom.insert(in);
}

void Odometry_C::load(char *filename){
	uint64_t time;	
	double roll,pitch,yaw;
	char str[30];
	Eigen::Vector3d pos;
	geometry_msgs::Quaternion orientation;

	io::CSVReader<8> file(filename);
	ref_csv.read_header(io::ignore_extra_column,
			"field.header.stamp",
			"field.pose.pose.position.x",
			"field.pose.pose.position.y",
			"field.pose.pose.position.z",
			"field.pose.pose.orientation.x",
			"field.pose.pose.orientation.y",
			"field.pose.pose.orientation.z",
			"field.pose.pose.orientation.w"
			);
	while(ref_csv.read_row(
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
		odom.emplace_back(time,pos,roll,pitch,yaw);
	}
}


