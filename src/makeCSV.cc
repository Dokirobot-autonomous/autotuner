#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <autotuner/save.h>

class odomData{
	public:
		void clear(void);
		void save(const char *filename);
		void insertFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &in) ;
		void insertLabel(void);
	private:
		const int MAX_ROW = 11;
		std::vector<std::string>data;
};

void odomData::clear(void){
	data.clear();
}
void odomData::save(const char *filename){
	int row=1;
	std::ofstream file(filename);
	
	for(int i=0;i<data.size();i++){
		file << data[i];
		if(row==MAX_ROW){
			file << '\n';
			row = 0;
		}
		else file << ',';
		
		row++;
	}
	
	file.close();
}
void odomData::insertFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &in) {
	char str[30];
	sprintf(str,"%lu%09lu",in->header.stamp.sec,in->header.stamp.nsec);
	data.push_back(str);
	data.push_back(std::to_string(in->header.seq));
	data.push_back(str);
	data.push_back(in->header.frame_id);
	data.push_back(std::to_string(in->pose.position.x));
	data.push_back(std::to_string(in->pose.position.y));
	data.push_back(std::to_string(in->pose.position.z));
	data.push_back(std::to_string(in->pose.orientation.x));
	data.push_back(std::to_string(in->pose.orientation.y));
	data.push_back(std::to_string(in->pose.orientation.z));
	data.push_back(std::to_string(in->pose.orientation.w));
}

void odomData::insertLabel(void){
	data.push_back("%time");
	data.push_back("field.header.seq");
	data.push_back("field.header.stamp");
	data.push_back("field.header.frame_id");
	data.push_back("field.pose.position.x");
	data.push_back("field.pose.position.y");
	data.push_back("field.pose.position.z");
	data.push_back("field.orientation.x");
	data.push_back("field.orientation.y");
	data.push_back("field.orientation.z");
	data.push_back("field.orientation.w");
}


//global variables
odomData odom;
bool done_signal = false;
time_t t_start,t_end;

struct range_t {
	double begin,end,level;
};
struct param_t {
	int nFeatures;
	double scaleFactor;
	int nLevels;
	int iniThFAST;
	int minThFAST;
};

void setParam(const param_t &in){
	ros::param::set("/orb_slam2_stereo/ORBextractor/nFeatures",in.nFeatures);
	ros::param::set("/orb_slam2_stereo/ORBextractor/scaleFactor",in.scaleFactor);
	ros::param::set("/orb_slam2_stereo/ORBextractor/iniThFAST",in.iniThFAST);
	ros::param::set("/orb_slam2_stereo/ORBextractor/minThFAST",in.minThFAST);
	ros::param::set("/orb_slam2_stereo/ORBextractor/nLevels",in.nLevels);
}
void printParam(param_t &in){
	printf("nFeatures:%d\n",in.nFeatures);
	printf("scaleFactor:%lf\n",in.scaleFactor);
	printf("iniThFAST:%d\n",in.iniThFAST);
	printf("minThFAST:%d\n",in.minThFAST);
	printf("nLevels:%d",in.nLevels);
}
void setRange(range_t &in,double begin,double end,double level){
	in.begin = begin;
	in.end = end;
	in.level = level;
}
void setParamList(
	range_t nf,
	range_t sf,
	range_t it,
	range_t mt,
	range_t nl,
	std::vector<param_t> &paramList){

	param_t tmp;
	double i,j,k,l,m;

	for(i=nf.begin;i<=nf.end;i+=nf.level){
		for(j=sf.begin;j<=sf.end;j+=sf.level){
			for(k=it.begin;k<=it.end;k+=it.level){
				for(l=mt.begin;l<=mt.end;l+=mt.level){
					for(m=nl.begin;m<=nl.end;m+=nl.level){
						tmp.nFeatures = i;
						tmp.scaleFactor = j;
						tmp.iniThFAST = k;
						tmp.minThFAST = l;
						tmp.nLevels = m;
						paramList.push_back(tmp);
					}
				}
			}
		}
	}

}


void getDoneSignal(const std_msgs::Empty &msg){
	printf("finsh rosbag\n");
	done_signal = true;
}
void callback(const geometry_msgs::PoseStampedConstPtr &topic){
	odom.insertFromPoseStamped(topic);	
	t_start = time(NULL);
}

void saveParam(const param_t &in,const char *filename){
	std::ofstream outfile(filename);
	outfile << "nFeatures,scaleFactor,nLevels,iniThFAST,minThFAST\n";
	outfile << in.nFeatures << ',' << in.scaleFactor << ',' << in.nLevels << ',' << in.iniThFAST << ',' << in.minThFAST << std::endl;
	outfile.close();
}
int main(int argc, char **argv){
	std::stringstream ss;
	std::string basefilename("orb_odom_");
	std::string bagfilename(argv[1]);
	std_msgs::String msg;
	int file_count = 0;
	range_t nf,sf,it,mt,nl; 
	std::vector<param_t> paramList;
	ros::init(argc, argv, "evaluater");

	ros::NodeHandle n;
	ros::Subscriber sub_pose = n.subscribe("/orb_slam2_stereo/lidar_pose",1000,callback);
	ros::Subscriber sub_done = n.subscribe("/done",1000,getDoneSignal);
	ros::Publisher pub_play_signal = n.advertise<std_msgs::String>("/play",1000);
	ros::ServiceClient callreset = n.serviceClient<std_srvs::Empty>("/orb_slam2_stereo/reset");
	std_srvs::Empty srv;
	ROS_INFO("start\n");
	ros::Rate loop_rate(10);

	setRange(nf,800,1600,200);
	setRange(sf,1.1,2.0  ,0.1);
	setRange(it,20,20  ,1);
	setRange(mt,7,7 ,1);
	setRange(nl,3,14,1);

	setParamList(nf,sf,it,mt,nl,paramList);
	printf("paramList.size:%d\n",paramList.size());
	printf("Press Enter\n");
	getchar();
	printParam(paramList[0]);
	setParam(paramList[0]); //parameter反映
	callreset.call(srv);//orb slam2初期化
	sleep(1);
	msg.data = bagfilename;//再生するbagfileを設定
	pub_play_signal.publish(msg);//bagfile再生

	odom.insertLabel();

//main loop
	while(ros::ok()){
		t_end = time(NULL);
		if(done_signal && t_end-t_start > 10){ //bagfileの再生が終わったら
			
			//csvファイル名を設定
			ss.str("");
			ss.clear(std::stringstream::goodbit);
			ss << basefilename << file_count << ".csv";
			odom.save(ss.str().c_str()); //odometryをcsvで保存

			ss.str("");
			ss.clear(std::stringstream::goodbit);
			ss << basefilename << "param_" << file_count << ".csv";
			saveParam(paramList[file_count],ss.str().c_str());

			//次の処理
			file_count++;
			odom.clear();
			odom.insertLabel();

			//parameterの反映
			printParam(paramList[file_count]);
			setParam(paramList[file_count]);
			callreset.call(srv);
			ROS_INFO("play bag\n");	
			pub_play_signal.publish(msg); //bagfileを再生
			done_signal = false;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
