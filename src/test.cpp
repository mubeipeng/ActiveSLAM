#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

//#include <tf/tf.h>
//#include <boost/foreach.hpp>
#include <fstream>

#include "planner.hpp"


class ActiveSlamNode{
private:
    Planner planner;
    std::map<int, LandmarkClass> landmarks;
    isam::Slam slam;

public:
    ActiveSlamNode():planner(landmarks){
        Eigen::MatrixXd m_odom = 10.*isam::eye(3);
        isam::Information noise_odom = isam::Information(m_odom);

        isam::Pose2d origin(0., 0., 0.);
        isam::Pose2d_Node* origin_node = new isam::Pose2d_Node();
        isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(origin_node, origin, noise_odom);
        slam.add_factor(prior);

        std::ifstream file_lm("/home/beipeng/Desktop/lm.txt");
        std::ifstream file_nb("/home/beipeng/Desktop/nb.txt");
        while(!file_lm.eof()){
            LandmarkClass lm;
            lm.isam_node = new isam::Pose2d_Node();
            double x,y,theta;
            int nb;
            file_lm>>lm.id>>lm.left_connected>>lm.right_connected>>x>>y>>theta;
            isam::Pose2d measure(x,y,theta);
            isam::Pose2d_Pose2d_Factor* lm_measure = new isam::Pose2d_Pose2d_Factor(origin_node, lm.isam_node,measure, noise_odom);
            slam.add_node(lm.isam_node);
            slam.add_factor(lm_measure);
            landmarks[lm.id]=lm;
        }
        while(!file_nb.eof()){
            int id1,id2;
            file_nb>>id1>>id2;
            landmarks[id1].neighbors.insert(std::pair<int,int>(id2,1));
        }
        file_lm.close();
        file_nb.close();
        slam.batch_optimization();

        for(double i=-isam::PI;i<isam::PI*2;i+=isam::PI/10){
            isam::Pose2d_Node* pose_node=new isam::Pose2d_Node;
            isam::Pose2d measure(0,0,i);
            isam::Pose2d_Pose2d_Factor* pose_factor = new isam::Pose2d_Pose2d_Factor(origin_node,  pose_node,measure, noise_odom);
            slam.add_factor(pose_factor);
            slam.add_node(pose_node);
            planner.AddNewAnchorPose(pose_node);
        }
    }

    ~ActiveSlamNode(){
    }

    void run();
};

void ActiveSlamNode::run(){
    Eigen::Vector3d pose;
    pose<<4,2,0;
    planner.setStartPose(pose);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(landmarks.size()*3,landmarks.size()*3);
    planner.generateNextGoal(pose,cov);
}

int main(int argc, char** argv){
    ActiveSlamNode active_slam;
    active_slam.run();
    return 0;
}
