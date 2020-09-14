#include <ros/ros.h>
#include <vector>
#include <utility> //pair
#include <math.h>
#include <tf/transform_listener.h>
#include <fstream> //ofstream
#include "csv.h"

#define MAX_EVALUATION_DISTANCE 1.0 

typedef std::pair<geometry_msgs::Pose,geometry_msgs::Pose> PairPose_t;
typedef std::vector<PairPose_t> PairPoseList_t;


struct pose_t {
	uint64_t stamp;
	geometry_msgs::Pose pose;
};

void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

geometry_msgs::Quaternion rpy_to_geometry_quat(double roll, double pitch, double yaw){
	tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
	geometry_msgs::Quaternion geometry_quat;
	quaternionTFToMsg(quat, geometry_quat);
	return geometry_quat;
}


void getMatchTimePair(
		std::vector<pose_t> &ref,
		std::vector<pose_t> &target,
		PairPoseList_t &list
		){

	int ref_index;
	int target_index;
	uint64_t ref_stamp,target_stamp;
	uint64_t delta,min;
	target_index = 0;
	for(ref_index=0;ref_index<ref.size();ref_index++){
		min = 0xFFFFFFFFFFFFFFFF;
		ref_stamp = ref[ref_index].stamp;
		for(target_index=0;target_index<target.size();target_index++){
			target_stamp = target[target_index].stamp;
			if(ref_stamp >= target_stamp) delta = ref_stamp - target_stamp;
			else delta = target_stamp - ref_stamp;
			if(delta<=min) min = delta;
			else {
				target_index--;
				break;
			}
		}
		//printf("index:%d\n",target_index);
		list.emplace_back(ref[ref_index].pose,target[target_index].pose);

	}

}
void position2vector(const geometry_msgs::Pose &in, tf::Vector3 &out){
	out.setX(in.position.x);
	out.setY(in.position.y);
	out.setZ(in.position.z);
}
double calSectionError(
		PairPoseList_t &list,
		int begin,
		int end){

	int i;
	double ret=0;
	double dummy_r,dummy_p,a_yaw,b_yaw,diff_yaw;
	tf::Vector3 a_origin,b_origin,a_abs_vec,b_abs_vec,a_rel_vec,b_rel_vec,tmp;
	position2vector(list[begin].first,a_origin);
	position2vector(list[begin].second,b_origin);

	GetRPY(list[begin].first.orientation,dummy_r,dummy_p,a_yaw);
	GetRPY(list[begin].second.orientation,dummy_r,dummy_p,b_yaw);
	diff_yaw = b_yaw - a_yaw;
	printf("a_yaw=%lf\n",a_yaw/(2*3.14)*360);
	printf("b_yaw=%lf\n",b_yaw/(2*3.14)*360);
	printf("diff_yaw=%lf\n",diff_yaw/(2*3.14)*360);
	//printf("---,,,\n");
	for(i=begin;i<=end;i++){
		position2vector(list[i].first,a_abs_vec);
		position2vector(list[i].second,b_abs_vec);
		a_rel_vec = a_abs_vec - a_origin;
		b_rel_vec = b_abs_vec - b_origin;
		tmp.setX(b_rel_vec.getX()*cos(-diff_yaw)-b_rel_vec.getY()*sin(-diff_yaw));
		tmp.setY(b_rel_vec.getX()*sin(-diff_yaw)+b_rel_vec.getY()*cos(-diff_yaw));
		tmp.setZ(b_rel_vec.getZ());
		//printf("%lf,%lf,%lf,%lf\n",a_rel_vec.getX(),a_rel_vec.getY(),b_rel_vec.getX(),b_rel_vec.getY());
		ret+= a_rel_vec.distance2(tmp);
	}

	ret = ret/(end-begin+1);//一点当たりの誤差を計算
	return ret;
}

void getErrorAndPoint(PairPoseList_t &list,std::vector<std::tuple<double,double,double>> &errorDataList,double distance){
	tf::Vector3 previous,current;
	double error;
	int i,j;
	distance *= distance;
	for(i=0;i<list.size();i=j){

		position2vector(list[i].first,previous);
		for(j=i+1;j<list.size();j++){
			position2vector(list[j].first,current);
			//printf("j:%d,dis:%lf\n",j,current.distance2(previous));

			if(distance <= current.distance2(previous)){
				error = calSectionError(list,i,j);
				errorDataList.emplace_back(list[(i+j)/2].first.position.x,list[(i+j)/2].first.position.y,error);
				break;
			}
		}
	}
}

void changeScalePose(const geometry_msgs::Pose &in, geometry_msgs::Pose &out,double scale){
	out.position.x = in.position.x * scale;
	out.position.y = in.position.y * scale;
	out.position.z = in.position.z * scale;
	out.orientation = in.orientation;
}

void changeAnglePose(const geometry_msgs::Pose &in,geometry_msgs::Pose &out,double yaw){
	double r,p,y;
	//GetRPY(in.orientation,r,p,y);
	//y = y+yaw;
	//out.orientation = rpy_to_geometry_quat(r, p, y);
	out.orientation = in.orientation;
	out.position.x = in.position.x*cos(yaw) - in.position.y*sin(yaw);
	out.position.y = in.position.x*sin(yaw) + in.position.y*cos(yaw);
	out.position.z = in.position.z;
}

int main(int argc, char **argv){
	if(argc < 6){
		printf("arg error\n");
		printf("example: ref.csv target.csv distance scale angle\n");
		return -1;
	}
	//ros::init(argc, argv, "evaluater");
	//ros::NodeHandle n;

	std::vector<pose_t> ref_pose,target_pose;
	pose_t tmp_pose;
	std::string str;
	PairPoseList_t match_list;

	std::ofstream outfile("out.csv");

	io::CSVReader<8> target_csv(argv[2]);
	io::CSVReader<8> ref_csv(argv[1]);
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


	geometry_msgs::Pose mod_pose;
	while(target_csv.read_row(
				str,
				tmp_pose.pose.position.x,
				tmp_pose.pose.position.y,
				tmp_pose.pose.position.z,
				tmp_pose.pose.orientation.x,
				tmp_pose.pose.orientation.y,	
				tmp_pose.pose.orientation.z,
				tmp_pose.pose.orientation.w)
			){
		tmp_pose.stamp = std::stoull(str); 
		//printf("%lu\n",tmp_pose.stamp);
		changeScalePose(tmp_pose.pose,mod_pose,atof(argv[4]));
		changeAnglePose(mod_pose,tmp_pose.pose,atof(argv[5]));
		target_pose.push_back(tmp_pose);
	}
	while(ref_csv.read_row(
				str,
				tmp_pose.pose.position.x,
				tmp_pose.pose.position.y,
				tmp_pose.pose.position.z,
				tmp_pose.pose.orientation.x,
				tmp_pose.pose.orientation.y,	
				tmp_pose.pose.orientation.z,
				tmp_pose.pose.orientation.w)
			){

		tmp_pose.stamp = std::stoull(str); 
		ref_pose.push_back(tmp_pose);
	}
	printf("ref_pose col:%zu\n",ref_pose.size());
	printf("target_pose col:%zu\n",target_pose.size());
/*	
	for(int i=0;i<target_pose.size();i++){
		double r,p,y;
		GetRPY(target_pose[i].pose.orientation,r,p,y);
		printf("yaw:%lf\n",y/2/3.14*360);
	}
	*/

	getMatchTimePair(ref_pose,target_pose,match_list);

	std::vector<std::tuple<double,double,double>> output;
	getErrorAndPoint(match_list,output,atof(argv[3]));

	outfile << "x,y,error\n";
	for(int i=0;i<output.size();i++){
		outfile << std::get<0>(output[i]) << ',' << std::get<1>(output[i]) << ',' << std::get<2>(output[i]) << std::endl;
	}
	outfile.close();
	return 0;
}
