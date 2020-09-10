#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <tf/transform_listener.h>
#include "csv.h"

#define MAX_EVALUATION_DISTANCE 3.0 

using namespace geometry_msgs;
using namespace std_msgs;

void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
double calError(tf::Vector3 &a,tf::Vector3 &b){
	double ret,ex,ey;
	ex = a.getX() - b.getX();
	ey = a.getY() - b.getY();
	ret = ex*ex + ey*ey;
	return ret;
}
/*
void callback(const OdometryConstPtr &topic1, const PoseStampedConstPtr &topic2){
	static double distance = 0.0;
	static int cnt=0;
	static double error = 0;
	static bool flag=false;
	PoseStamped orb_lidar_pos;
	PointStamped div_lio_pos, div_orb_pos;

	div_lio_pos.header.frame_id = "map";
	div_orb_pos.header.frame_id = "map";
	

	getLidarPose(topic2,&orb_lidar_pos);

	tf::Vector3 init_lio_pose,init_orb_pose,lio_pose,orb_pose,lio_relative_pose,orb_relative_pose;
	tf::Vector3 previous_lio_pose;
	lio_pose = tf::Vector3(topic1->pose.pose.position.x,topic1->pose.pose.position.y,0.0);
	orb_pose = tf::Vector3(orb_lidar_pos.pose.position.x,orb_lidar_pos.pose.position.y,0.0); 
	//orbのカメラ座標からlidarの座標を算出
	output_pub.publish(orb_lidar_pos);
	if(!flag){
		div_lio_pos.point.x = lio_pose.getX();
		div_lio_pos.point.y = lio_pose.getY();
		div_lio_pos.point.z = 0.0;
		div_orb_pos.point.x = orb_pose.getX();
		div_orb_pos.point.y = orb_pose.getY();
		div_orb_pos.point.z = 0.0;
		output_lio_division.publish(div_lio_pos);
		output_orb_division.publish(div_orb_pos);
		init_lio_pose = lio_pose; 
		init_orb_pose = orb_pose;
		previous_lio_pose = lio_pose;
		flag = true;
	}
	if(distance<MAX_EVALUATION_DISTANCE){
		if(cnt%5==0)distance += lio_pose.distance2(previous_lio_pose);
		cnt++;
		previous_lio_pose = lio_pose;
		lio_relative_pose = lio_pose-init_lio_pose;
		orb_relative_pose = orb_pose-init_orb_pose;
		error += calError(lio_relative_pose,orb_relative_pose);		
	}
	else{

		printf("%lf\n",error);
		error=0.0;
		cnt=0;
		distance=0.0;
		flag=false;
	}



}
*/
struct pose_t {
	uint64_t stamp;
	geometry_msgs::Pose pose;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;
	std::vector<pose_t> ref_pose,target_pose;
	pose_t tmp_pose;

	io::CSVReader<8> target_csv("test.csv");
	target_csv.read_header(io::ignore_extra_column,
	"field.header.stamp",
	"field.pose.position.x",
	"field.pose.position.y",
	"field.pose.position.z",
	"field.orientation.x",
	"field.orientation.y",
	"field.orientation.z",
	"field.orientation.w"
	);

	while(target_csv.read_row(
		tmp_pose.stamp,
		tmp_pose.pose.position.x,
		tmp_pose.pose.position.y,
		tmp_pose.pose.position.z,
		tmp_pose.pose.orientation.x,
		tmp_pose.pose.orientation.y,	
		tmp_pose.pose.orientation.z,
		tmp_pose.pose.orientation.w)
		){
		
		ref_pose.push_back(tmp_pose);
	}
	printf("%zu\n",ref_pose.size());
	ROS_INFO("start\n");

	ros::spin();
	return 0;
}
