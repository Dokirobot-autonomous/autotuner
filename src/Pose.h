#include <Eigen/Core>
#include <Eigen/Geometry>
class Pose_C{
	private:
		uint64_t time;
		Eigen::Vector3d pos;
		double roll,pitch,yaw;
	public:
		Pose_C(uint64_t time_,Eigen::Vector3d pos_,double roll_,double pitch_,double yaw_);
		rotate(double roll_,double pitch_,double yaw_);
		print(void);
}
