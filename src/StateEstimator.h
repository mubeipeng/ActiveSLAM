#ifndef STATEESTIMATOR_H
#define STATEESTIMATOR_H

#include <eigen3/Eigen/Dense>
#include <math.h>
#include "map_processor.hpp"

class StateEstimator{
public:

    StateEstimator(std::map<int, LandmarkClass>&);

    ~StateEstimator();

    Eigen::Vector3d getState(){
        return current_state;
    }

    Eigen::Matrix3d getCovariance(){
        return current_cov;
    }

    void setStart(Eigen::Vector3d initState);

    Eigen::Vector3d propagate(Eigen::Vector3d dpos);
    Eigen::Vector3d tagDetectsCallback(std::vector<AprilTags::TagDetection> tagdetects);
    void update(Eigen::Vector4d obs);


private:
    // landmarks
    std::map<int, LandmarkClass>& landmarks;

    // current state
    Eigen::Vector3d current_state;
    Eigen::Matrix3d current_cov;
    bool state_lock;

    // noise
    Eigen::Matrix3d noise_odom;
    Eigen::Matrix3d noise_obs;
    Eigen::Matrix3d noise_process;
};

#endif // PATH_FOLLOWER_H
