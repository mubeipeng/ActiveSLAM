//#include <rosbag/bag.h>
//#include <rosbag/view.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

//#include <tf/tf.h>
//#include <boost/foreach.hpp>

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

#include "map_processor.hpp"
#include "planner.hpp"
#include "StateEstimator.h"

class ActiveSlamNode{
private:
    MapProcessor map_proc;
    Planner planner;
    StateEstimator state_estimator;
    std::map<int, LandmarkClass> landmarks;
    bool landmark_locked;
    
    // camera parameters
    float* depth_data;
    int image_width, image_height;
    bool depth_initialized;
    AprilTags::TagDetector* m_tagDetector;
    double fx,fy,px,py,tagsize;
    ros::Time lastAprilUpdate;

    // state track
    int robot_mode;
    isam::Pose2d last_pose;
    double explore_angle_turned;

    // log files
    std::ofstream log;
    
    // ros topics
    ros::NodeHandle nh;
    ros::Publisher visualization_pub;
    ros::Publisher state_pub;
    ros::Publisher mode_pub;
    ros::Publisher path_pub;


    // private functions
    Eigen::MatrixXd depthToLaser();
    std::vector<AprilTags::TagDetection> DetectAprilTags(const sensor_msgs::ImageConstPtr& image_msg);
    void publishMap(std_msgs::Header header);
    void publishState(Eigen::Vector3d, std_msgs::Header header);
    void publishControl(int);
    void publishPath(std::list<isam::Point2d> path);

public:
    ActiveSlamNode(ros::NodeHandle nh):map_proc(landmarks),planner(landmarks),state_estimator(landmarks){
        m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);
        tagsize = 0.164;
	depth_initialized = false;
	
	lastAprilUpdate = ros::Time::now();

	landmark_locked=false;

        robot_mode = 0;
        explore_angle_turned = 0;
	last_pose.set(0,0,0);        
	
        //visualization_pub = nh.advertise<visualization_msgs::MarkerArray>("active_slam/topology_feature_map", 1);
        //state_pub = nh.advertise<geometry_msgs::PoseStamped>("active_slam/state_estimate",10);
	//mode_pub = nh.advertise<std_msgs::Float64MultiArray>("/active_slam/modeCmd",10);
	//path_pub = nh.advertise<std_msgs::Float64MultiArray>("active_slam/path",10);
	visualization_pub = nh.advertise<visualization_msgs::MarkerArray>("/TB01/topology_feature_map", 1);
        state_pub = nh.advertise<geometry_msgs::PoseStamped>("/TB01/pose",1);
	mode_pub = nh.advertise<std_msgs::Float64MultiArray>("/TB01/carCmd",1);
	path_pub = nh.advertise<std_msgs::Float64MultiArray>("/TB01/path",1);
    }

    ~ActiveSlamNode(){
        delete m_tagDetector;        
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& image_info_msg);

    void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg);          

    void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

    void OdomCallback(nav_msgs::Odometry::ConstPtr msg);

};

Eigen::MatrixXd ActiveSlamNode::depthToLaser(){
    Eigen::MatrixXd laser(image_width,2);
    for(int j=0;j<image_width;j++){
        laser(j,0)=100;

        for(int i=0;i<image_height;i++){
            float z = depth_data[i*image_width+j];
            float x = (j-px)/fx*z;
            float y = (i-py)/fy*z;
            if(y>=-0.3 && y <0.1 && z>0.05){
                if(z<laser(j,0)){
                    laser(j,0)=z;
                    laser(j,1)=-x;
                }
            }
        }
        if(laser(j,0)==100){
            laser(j,0)=-1;
            laser(j,1)=0;
        }
    }
    return laser;
}

std::vector<AprilTags::TagDetection> ActiveSlamNode::DetectAprilTags(const sensor_msgs::ImageConstPtr& image_msg){

    cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(image_msg,image_msg->encoding);
    const cv::Mat& image_gray   = rgb_ptr->image;
    //cv::Mat image_gray;
    //cv::cvtColor(image_rgb, image_gray, CV_BGR2GRAY);

    std::vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);

    for(int i=0;i<detections.size();i++){
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detections[i].getRelativeTranslationRotation(tagsize, fx, fy, px, py,translation, rotation);
        double roll,pitch,yaw;
        isam::Rot3d::wRo_to_euler(rotation,yaw,pitch,roll);
        detections[i].hxy = std::pair<float,float>(translation(0),translation(1));
        detections[i].observedPerimeter = -pitch;
        //detections[i].draw(image_gray);
    }
    //cv::imshow("tag detections",image_gray);cv::waitKey(10);
    return detections;
 }

void ActiveSlamNode::publishState(Eigen::Vector3d state, std_msgs::Header header){
    geometry_msgs::PoseStamped msg;
    msg.header = header;
    msg.pose.position.x = state(0);
    msg.pose.position.y = state(1);
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = std::sin(0.5*state(2));
    msg.pose.orientation.w = std::cos(0.5*state(2));

    //std::cout<<"[active_slam_node] state"<< state.transpose()<<std::endl;
    state_pub.publish(msg);
}

void ActiveSlamNode::publishControl(int angle_multiplier){
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(1);
    msg.data.push_back(0);
    msg.data.push_back(0.2*angle_multiplier);
    mode_pub.publish(msg);
}

void ActiveSlamNode::publishPath(std::list<isam::Point2d> path){
    std_msgs::Float64MultiArray mode_msg;
    mode_msg.data.push_back(0);
    mode_pub.publish(mode_msg);

    std_msgs::Float64MultiArray path_msg;
    path_msg.layout.data_offset=0;
    
    std_msgs::MultiArrayDimension dim;
    dim.size = path.size();
    dim.stride = dim.size*4;
    path_msg.layout.dim.push_back(dim);

    dim.size=4;
    dim.stride = 4;
    path_msg.layout.dim.push_back(dim);

    std::cout<<"new path";
    for(std::list<isam::Point2d>::iterator it=path.begin();it!=path.end();it++){
        path_msg.data.push_back(it->x());
        path_msg.data.push_back(it->y());
        path_msg.data.push_back(0);
        path_msg.data.push_back(0.2);
        std::cout<<" ->("<<it->x()<<","<<it->y()<<")";
    }
    std::cout<<std::endl;
    path_pub.publish(path_msg);

}

void ActiveSlamNode::publishMap(std_msgs::Header header){
    visualization_msgs::Marker marker_lm, marker_lines;
    marker_lm.header = header;
    marker_lm.ns = "landmark";
    marker_lm.type = visualization_msgs::Marker::CUBE_LIST;
    marker_lm.action = visualization_msgs::Marker::ADD;
    marker_lm.pose.orientation.x = 0.0;
    marker_lm.pose.orientation.y = 0.0;
    marker_lm.pose.orientation.z = 0.0;
    marker_lm.pose.orientation.w = 1.0;
    marker_lm.scale.x = 0.1;
    marker_lm.scale.y = 0.1;
    marker_lm.scale.z = 0.1;
    marker_lm.color.a = 1.0; // Don't forget to set the alpha!
    marker_lm.color.r = 0.0;
    marker_lm.color.g = 1.0;
    marker_lm.color.b = 0.0;

    marker_lines.header = header;
    marker_lines.ns = "lines";
    marker_lines.type = visualization_msgs::Marker::LINE_LIST;
    marker_lines.action = visualization_msgs::Marker::ADD;
    marker_lines.pose.orientation.x = 0.0;
    marker_lines.pose.orientation.y = 0.0;
    marker_lines.pose.orientation.z = 0.0;
    marker_lines.pose.orientation.w = 1.0;
    marker_lines.scale.x = 0.1;
    marker_lines.scale.y = 0.1;
    marker_lines.scale.z = 0.1;
    marker_lines.color.a = 1.0; // Don't forget to set the alpha!
    marker_lines.color.r = 0.0;
    marker_lines.color.g = 0.0;
    marker_lines.color.b = 1.0;

    for(std::map<int,LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        geometry_msgs::Point p1;
	p1.x = it->second.isam_node->value().x();
	p1.y = it->second.isam_node->value().y();
	p1.z = 0;
	
	marker_lm.points.push_back(p1);

        for(std::map<int,int>::iterator it_nb = it->second.neighbors.begin();it_nb!=it->second.neighbors.end();it_nb++){
	    geometry_msgs::Point p2;		
            p2.x = landmarks[it_nb->first].isam_node->value().x();
            p2.y = landmarks[it_nb->first].isam_node->value().y();
            p2.z = 0;
            
            marker_lines.points.push_back(p1);
	    marker_lines.points.push_back(p2);
        }
    }

    visualization_msgs::MarkerArray feature_map;
    feature_map.markers.push_back(marker_lm);
    feature_map.markers.push_back(marker_lines);
    
    visualization_pub.publish(feature_map);
}

void ActiveSlamNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& image_info_msg){
    fx = (double)image_info_msg->K[0];
    px = (double)image_info_msg->K[2];
    fy = (double)image_info_msg->K[4];
    py = (double)image_info_msg->K[5];
}
      

void ActiveSlamNode::depthCallback(const sensor_msgs::ImageConstPtr& msg){
    if(!depth_initialized){
	image_width = msg->width;
	image_height = msg->height;
	depth_data = new float[image_width*image_height];
	depth_initialized = true;
    }
    const unsigned short int* depth_mm = reinterpret_cast<const unsigned short int*>(msg->data.data()); // depth in millimeters
    for(int i=0;i<image_width*image_height;i++){
        if( *(depth_mm+i)==0){
	    depth_data[i] = NAN;
	}else{
	    depth_data[i]=*(depth_mm + i)*0.001;
	}
    }    
}

void ActiveSlamNode::imageCallback(const sensor_msgs::ImageConstPtr& image_msg){
    // skip landmark is locked in other threads
    //if(landmark_locked) return;
    // do nothing of depth not initialized    
    if(!depth_initialized) return;
    //update only when last update is 1sec ago
    if( (ros::Time::now()-lastAprilUpdate).toSec()<0.3 ) return;
    
    //landmark_locked=true;    
    lastAprilUpdate = ros::Time::now();

    Eigen::MatrixXd laser = depthToLaser();
    std::vector<AprilTags::TagDetection> tagdetects = DetectAprilTags(image_msg);
    
    //switch(robot_mode){
    //case 0:
    planner.AddNewAnchorPose(map_proc.updateFeature(laser,tagdetects));
    if(robot_mode == 2 && !planner.checkPathValidity()){
	robot_mode = 0;
	std::cout<<"[active_slam_node] now observed obstacle blocked old path, replan"<<std::endl;
    }
		
    publishMap(image_msg->header);
    
    //    break;
    //case 2:
    //    state_estimator.tagDetectsCallback(tagdetects);
    //    break;
    //}
    //landmark_locked=false;
}

void ActiveSlamNode::OdomCallback(nav_msgs::Odometry::ConstPtr msg){
    //if(landmark_locked) return;

    //landmark_locked=true;
    
    /**
      * initialize time and pose;        
     */
    if( last_pose.vector().norm()==0){
        last_pose.set(msg->pose.pose.position.x,
                     msg->pose.pose.position.y,
                     2*std::asin(msg->pose.pose.orientation.z));
        return;
    }

    /**
      * compute odometry change     
     */
    double yaw = 2*std::asin(msg->pose.pose.orientation.z);
    isam::Pose2d current_pose(msg->pose.pose.position.x,msg->pose.pose.position.y,yaw );
    isam::Pose2d delpos = current_pose.ominus(last_pose);
    if(delpos.t()>isam::PI) delpos.set_t(delpos.t()-isam::PI*2);
    if(delpos.t()<-isam::PI) delpos.set_t(delpos.t()+isam::PI*2);
    last_pose = current_pose;

    /**
      * process odoemtry measurement based on exploration or path_following mode
     */
    //std::cout<<"odom OdomCallback "<<robot_mode<<"   "<<explore_angle_turned<<std::endl;
    Eigen::Vector3d current_state;
    switch(robot_mode){
    case 0:
        /**
          * exploring mode
         */
        current_state = map_proc.updateOdom(delpos);
        explore_angle_turned += delpos.t();
        if(std::abs(explore_angle_turned)>2*isam::PI){
            explore_angle_turned = 0;
            robot_mode = 1;
            isam::Pose2d startpose;
            Eigen::MatrixXd cov;
            map_proc.updateMap(startpose,cov);
            planner.generateNextGoal(startpose,cov);
            //std::cout<<"[active_slam node] finished exploring, map updated at" << current_state.transpose()<<std::endl;        
        }else{
            publishControl(1);
        }
        break;

    case 1:
        /**
          * turn towards goal direction
          */
        current_state = map_proc.updateOdom(delpos);
	int angle_multiplier;
	angle_multiplier = planner.headRightAngle(current_state);
        if(angle_multiplier==0){
            robot_mode = 2;
            planner.setStartPose(current_state);
            //state_estimator.setStart(current_state);
            std::list<isam::Point2d> path = planner.getPath();
            publishPath(path);
            std::cout<<"[active_slam_node] heading to goal"<<std::endl;
        }else{
            publishControl(angle_multiplier);
        }
        break;

    case 2:
        /**
          * path floowing mode
         */
        current_state = map_proc.updateOdom(delpos);
        if(planner.goalReached(current_state)){
            robot_mode = 0;
	    std::cout<<"[active_slam_node] goal reached, swtich back to exploration mode"<<std::endl;
	    publishControl(1);
            //map_proc.updateOdom(isam::Pose2d(current_state(0),current_state(1),current_state(2)).ominus(planner.getStartPose()));
        }        
        break;
    }    
    publishState(current_state,msg->header);        
    //landmark_locked=false;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ActiveSlamNode");
    ros::NodeHandle nh; 	
    ActiveSlamNode active_slam(nh);
    
    ros::Subscriber image_sub=nh.subscribe("/camera/rgb/image", 1, &ActiveSlamNode::imageCallback, &active_slam);
    ros::Subscriber depth_sub=nh.subscribe("/camera/depth/image", 1, &ActiveSlamNode::depthCallback, &active_slam);
    ros::Subscriber image_info_sub=nh.subscribe("/camera/rgb/camera_info", 1,&ActiveSlamNode::cameraInfoCallback, &active_slam);
    ros::Subscriber odom_sub=nh.subscribe("/odom", 10,&ActiveSlamNode::OdomCallback,&active_slam);

    ROS_INFO("active_slam_node initialized, start exploring...");
    ros::spin();
    return 0;
}
