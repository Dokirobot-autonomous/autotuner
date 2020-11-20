#include "Odometry.h"
#include <iostream>
#include <string>
double calError(Odometry_C &ref, Odometry_C &tar){
	Eigen::Vector3d delta;
	double ret=0;
	for(int i=0;i<tar.pose.size();i++){
		delta = ref.pose[i].getPos()-tar.pose[i].getPos();
		ret += delta.norm();
	}
	return ret;
}
int main(int argc, char *argv[]){
	double result;
	std::string base_file_name(argv[2]);
	std::string filename;

	int file_idx,max_file_num;
	max_file_num = atoi(argv[3]);
	Odometry_C ref,target;
	ref.load(argv[1],TYPE_ODOMETRY,2300);

	std::cout << "filenum,error" << std::endl;


	for(file_idx=1;file_idx<=max_file_num;file_idx++){
		filename = base_file_name + std::to_string(file_idx) + std::string(".csv");
		target.reset();
		target.load(filename.c_str(),TYPE_POSE);
		std::cout << "length" << target.length() << std::endl;
		target.extract(ref);
		if(target.pose.size()==ref.pose.size()){
			target.rotatePos(0,0,strtod(argv[4],NULL));
			result = calError(ref,target);	
			std::cout << filename << ',' << result << std::endl;
		}
		//else std::cout << "too short" << std::endl;
	}
	return 0;
}
