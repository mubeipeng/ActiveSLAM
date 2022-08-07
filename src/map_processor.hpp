/**
 *  @file aprilTags_proc.hpp
 *  @author Matthew Graham <mcgraham@mit.edu>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2014, Massachusetts Institute of Technology
**/ 

#ifndef APRIL_TAGS_PROC_H
#define APRIL_TAGS_PROC_H

// April Tags specific includes
#include "AprilTags/TagDetection.h"

// slam
#include "isam/isam.h"

//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <cmath>
#include <vector>
#include <map>

struct LandmarkClass{
    int id;
    bool left_connected, right_connected;
    //float left_size=0,right_size=0;
    isam::Pose2d_Node* isam_node;
    std::map<int,int> neighbors;
};

class MapProcessor{
public:

    /** @brief Constructor from ROS nodehandles
     */
    MapProcessor(std::map<int, LandmarkClass>& landmarks);

    /** @brief Default destructor
     */
    ~MapProcessor();

    /** @brief process an odometry measurement
     */
    Eigen::Vector3d updateOdom(isam::Pose2d odom_reading);
    /**
     * @brief process new feature detections: add a new slam pose node and connect to detected feature nodes
     * @param laser
     * @param tagdetects
     */
    isam::Pose2d_Node* updateFeature(Eigen::MatrixXd laser, std::vector<AprilTags::TagDetection> tagdetects);

    /**
     * @brief run slam solver to update feature positions, and return the current robot pose estimate
     */
    void updateMap(isam::Pose2d&, Eigen::MatrixXd&);
    
private:
    std::map<int, LandmarkClass>& landmarks;
    //slam solver    
    isam::Slam slam;
    isam::Noise noise_odom;
    isam::Noise noise_lm;
    isam::Pose2d_Node* last_pose;
    isam::Pose2d delpos;

    //unsigned int batch_every;
    //unsigned int step;
    
    // private functions
    /** @brief segment depth image
     */
    void updateTopology(Eigen::MatrixXd laser,std::vector<AprilTags::TagDetection> tagdetects);     
};

#endif
