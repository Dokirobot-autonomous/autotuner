#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "Pose.h"
#include <vector>
enum FILETYPE{
	TYPE_ODOMETRY,
	TYPE_POSE
};
class Odometry_C{
	private:
	public:
		std::vector<Pose_C> pose;
		void load(char *filename,FILETYPE type);
		void print(bool flag);
		void extract(const Odometry_C &ref);
		void insert(const Pose_C &in);
};
#endif


