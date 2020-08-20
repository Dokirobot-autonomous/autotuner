#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace geometry_msgs;
using namespace message_filters;

void callback(const PoseStamped pose1, const PoseStamped pose2){
	printf("%d,%d\n",pose1.pose.position.x,pose1.pose.position.y);
}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;

	message_filters::Subscriber<PoseStamped> pose1_sub(n,"pose1",1);
	message_filters::Subscriber<PoseStamped> pose2_sub(n,"pose2",1);

	typedef sync_policies::ApproximateTime<PoseStamped,PoseStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose1_sub,pose2_sub);
	sync.registerCallback(&callback);


	ros::spin();
	return 0;
}
