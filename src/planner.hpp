#ifndef PLANNER_H
#define PLANNER_H

#include "isam/isam.h"
#include "map_processor.hpp"
#include <geometry_msgs/Twist.h>

class Planner{
public:
    Planner(std::map<int, LandmarkClass>&);

    void AddNewAnchorPose(isam::Pose2d_Node*);

    void generateNextGoal(isam::Pose2d start_pos, Eigen::MatrixXd covariance);

    bool checkPathValidity();

    bool goalReached(Eigen::Vector3d current_pose);

    int headRightAngle(Eigen::Vector3d current_pose);

    isam::Pose2d getStartPose(){return start_pose;}

    void setStartPose(Eigen::Vector3d sp){start_pose.set(sp(0),sp(1),sp(2));}

    std::list<isam::Point2d> getPath(){return path_to_follow;}


private:
    std::map<int, LandmarkClass>& landmarks;
    double landmark_size;
    double x_min,x_max,y_min,y_max; //the environment size    

    std::list<isam::Pose2d_Node*> pose_history; //robot pose history
    std::list<isam::Point2d> path_to_follow; // next goal
    isam::Pose2d start_pose;
    double goal_reach_threshold;

    int no_samples;
    double camera_view_distance;
    double unknown_landmark_density;

    bool checkVisibility(isam::Pose2d p1, isam::Point2d p2);
    bool checkVisibility(double x1,double y1,double x2,double y2);
    bool checkFarFromObstacle(double x1, double y1);
    double exploreBonus(std::vector<LandmarkClass> lms, isam::Point2d pose);
};

#endif // PLANNER_H
