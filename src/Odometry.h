#include "Pose.h"
class Odometry_C{
	private:
		std::vector<Pose_C> odom;
	public:
		void load(char *filename);
		void insert(Pose_C &in);

}


