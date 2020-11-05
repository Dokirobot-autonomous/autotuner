#ifndef POSE_H
#define POSE_H
#include <Eigen/Core>
#include <Eigen/Geometry>
class Pose_C{
	private:
		uint64_t time;
		Eigen::Vector3d pos;
		double roll,pitch,yaw;
	public:
		Pose_C(uint64_t time_,Eigen::Vector3d pos_,double roll_,double pitch_,double yaw_);
		void rotate(double roll_,double pitch_,double yaw_);
		void scale(double scale);
		void print(void);
		uint64_t getTime(void) const { return time;}
		Eigen::Vector3d getPos(void){ return pos;}
};
#endif
