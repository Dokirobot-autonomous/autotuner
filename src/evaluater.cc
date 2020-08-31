#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace std_msgs;
ros::Publisher output_pub; 
/*
void transformTolioCoordinate(Pose &in,Pose &out,double yaw){
	out.pose.x = in.pose.x * cos(yaw) - in.pose.y * sin(yaw);
	out.pose.y = in.pose.x * sin(yaw) + in.pose.y * cos(yaw);
	out.pose.z = 0.0;
}
*/

//camera座標系におけるlidar座標を取得
void getLidarPose(const PoseStampedConstPtr &in, PoseStamped *out){
	double yaw;
	const double d = 0.33;
	Quaternion q = in->pose.orientation;
	out->header = in->header;
	yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

	out->pose.position.x = in->pose.position.x - d*cos(yaw);
	out->pose.position.y = in->pose.position.y - d*sin(yaw);
	out->pose.position.z = 0.0;
	out->pose.orientation = q;
}
//lidarの初期位置を原点とした座標系におけるlidarの座標を計算
void getLidarPose_map(const PoseStamped *in, PoseStamped *out){
	const double d = 0.33;
	*out = *in;
	out->header.frame_id = "map";
	out->pose.position.x = in->pose.position.x + d;
}
void callback(const OdometryConstPtr &pose1, const PoseStampedConstPtr &pose2){
	ROS_INFO("sub\n");
	PoseStamped orb_pos,orb_pos_map;
	//ORB座標をlio座標に変換
	getLidarPose(pose2,&orb_pos);
	getLidarPose_map(&orb_pos,&orb_pos_map);
//	output_pub.publish(orb_pos_map);

	ROS_INFO("lio mapping:\n%lf,%lf\n",pose1->pose.pose.position.x,pose1->pose.pose.position.y);
	ROS_INFO("secs:%u\n",pose1->header.stamp.sec);
	ROS_INFO("nsecs:%u\n",pose1->header.stamp.nsec);
	ROS_INFO("orb slam2:\n%lf,%lf\n",pose2->pose.position.x,pose2->pose.position.y);
	ROS_INFO("secs:%u\n",pose2->header.stamp.sec);
	ROS_INFO("nsecs:%u\n",pose2->header.stamp.nsec);

}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;

//	output_pub = n.advertise<PoseStamped>("lidar_pose",10);
	message_filters::Subscriber<Odometry> pose_ref(n,"/aft_mapped_to_init",1);
	message_filters::Subscriber<PoseStamped> pose_target(n,"/orb_slam2_stereo/pose",1);

	typedef sync_policies::ApproximateTime<Odometry,PoseStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_ref,pose_target);
	sync.registerCallback(boost::bind(&callback,_1,_2));

	ROS_INFO("test\n");

	ros::spin();
	return 0;
}
