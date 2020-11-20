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
		void load(const char *filename,FILETYPE type,int limit = 100000);
		void save(char *filename);
		void print(bool flag);
		void extract(const Odometry_C &ref);
		void insert(const Pose_C &in);
		void rotatePos(double r,double p,double y);
		void reset(void);
		double length(void);
};
#endif


