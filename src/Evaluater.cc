#include "Odometry.h"
#include <iostream>
int main(int argc, char *argv[]){
	Odometry_C ref,target;
	ref.load(argv[1],TYPE_ODOMETRY);
	target.load(argv[2],TYPE_POSE);
	ref.print(false);
	target.print(false);
	std::cout << "---------" << std::endl;
	target.extract(ref);
	target.print(false);
	
	return 0;
}
