#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <tf/transform_listener.h>
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace std_msgs;
ros::Publisher output_pub; 

void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
//camera座標系におけるlidar座標を取得
void getLidarPose(const PoseStampedConstPtr &in, PoseStamped *out){
	double r,p,y;
	const double d = 0.33;
	Quaternion q = in->pose.orientation;
	out->header = in->header;
	out->header.frame_id = "map";

	GetRPY(q,r,p,y);

	out->pose.position.x = in->pose.position.x - d*cos(y)+d;
	out->pose.position.y = in->pose.position.y - d*sin(y);
	out->pose.position.z = 0.0;
	out->pose.orientation = q;
}
void callback(const OdometryConstPtr &pose1, const PoseStampedConstPtr &pose2){
	PoseStamped orb_lidar_pos;
	ROS_INFO("sub\n");
	//orbのカメラ座標からlidarの座標を算出
	getLidarPose(pose2,&orb_lidar_pos);
	output_pub.publish(orb_lidar_pos);

}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;

	output_pub = n.advertise<PoseStamped>("/autotuner/lidar_pose",10);
	message_filters::Subscriber<Odometry> pose_ref(n,"/aft_mapped_to_init",1);
	message_filters::Subscriber<PoseStamped> pose_target(n,"/orb_slam2_stereo/pose",1);

	typedef sync_policies::ApproximateTime<Odometry,PoseStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_ref,pose_target);
	sync.registerCallback(boost::bind(&callback,_1,_2));

	ROS_INFO("start\n");

	ros::spin();
	return 0;
}
