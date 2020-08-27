#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;

void callback(const OdometryConstPtr &pose1, const PoseStampedConstPtr &pose2){
	printf("lio mapping:%lf,%lf\n",pose1->pose.pose.position.x,pose1->pose.pose.position.y);
	printf("orb slam2:%lf,%lf\n",pose2->pose.position.x,pose2->pose.position.y);
}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;

	message_filters::Subscriber<Odometry> pose_ref(n,"/aft_mapped_to_init",1);
	message_filters::Subscriber<PoseStamped> pose_target(n,"/orb_slam2_stereo/pose",1);

	typedef sync_policies::ApproximateTime<Odometry,PoseStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_ref,pose_target);
	sync.registerCallback(boost::bind(&callback,_1,_2));


	ros::spin();
	return 0;
}
