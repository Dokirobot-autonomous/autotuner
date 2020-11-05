#include "Pose.h"
#include <iostream>
Pose_C::Pose_C(uint64_t time_,Eigen::Vector3d pos_,double roll_,double pitch_,double yaw_){
	time = time_;
	pos = pos_;
	roll = roll_;
	pitch = pitch_;
	yaw = yaw_;
}


void Pose_C::rotate(double roll_,double pitch_,double yaw_){
	
}

void Pose_C::scale(double scale){
	pos = pos*scale;
}

void Pose_C::print(void){
	std::cout << "time = " << time << std::endl;
	std::cout << "pos = \n" << pos << std::endl;
	std::cout << "RPY = \n" << roll << ',' << pitch << ',' << yaw << std::endl;
}
