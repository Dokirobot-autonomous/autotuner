#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <tf/transform_listener.h>

#define MAX_EVALUATION_DISTANCE 1.0 

using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace std_msgs;
ros::Publisher output_pub;
ros::Publisher output_division;

void GetRPY(const geometry_msgs::Quaternion &q,
		double &roll,double &pitch,double &yaw){
	tf::Quaternion quat(q.x,q.y,q.z,q.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
//map座標系におけるlidar座標を取得
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
double calError(tf::Vector3 &a,tf::Vector3 &b){
	double ret,ex,ey;
	ex = a.getX() - b.getX();
	ey = a.getY() - b.getY();
	ret = ex*ex + ey*ey;
	return ret;
}
void callback(const OdometryConstPtr &topic1, const PoseStampedConstPtr &topic2){
	static double distance = 0.0;
	static double error = 0;
	static bool flag=false;
	PoseStamped orb_lidar_pos;
	PointStamped div_pos;

	div_pos.header.frame_id = "map";
	

	getLidarPose(topic2,&orb_lidar_pos);

	tf::Vector3 init_lio_pose,init_orb_pose,lio_pose,orb_pose,lio_relative_pose,orb_relative_pose;
	tf::Vector3 previous_lio_pose;
	lio_pose = tf::Vector3(topic1->pose.pose.position.x,topic1->pose.pose.position.y,0.0);
	orb_pose = tf::Vector3(topic2->pose.position.x,topic2->pose.position.y,0.0); 
	//orbのカメラ座標からlidarの座標を算出
	output_pub.publish(orb_lidar_pos);
	if(!flag){
		div_pos.point.x = lio_pose.getX();
		div_pos.point.y = lio_pose.getY();
		div_pos.point.z = 0.0;
		output_division.publish(div_pos);
		init_lio_pose = lio_pose; 
		init_orb_pose = orb_pose;
		previous_lio_pose = lio_pose;
		flag = true;
	}
	if(distance<MAX_EVALUATION_DISTANCE){
		distance += lio_pose.distance2(previous_lio_pose);
		previous_lio_pose = lio_pose;
		lio_relative_pose = lio_pose-init_lio_pose;
		orb_relative_pose = orb_pose-init_orb_pose;
		error += calError(lio_relative_pose,orb_relative_pose);		
	}
	else{

		printf("error:%lf\n",error);
		error=0.0;
		distance=0.0;
		flag=false;
	}



}
int main(int argc, char **argv){
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;

	output_pub = n.advertise<PoseStamped>("/autotuner/lidar_pose",10);
	output_division = n.advertise<PointStamped>("/autotuner/division",10);
	message_filters::Subscriber<Odometry> pose_ref(n,"/aft_mapped_to_init",1);
	message_filters::Subscriber<PoseStamped> pose_target(n,"/orb_slam2_stereo/pose",1);

	typedef sync_policies::ApproximateTime<Odometry,PoseStamped> MySyncPolicy;

	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),pose_ref,pose_target);
	sync.registerCallback(boost::bind(&callback,_1,_2));

	ROS_INFO("start\n");

	ros::spin();
	return 0;
}
