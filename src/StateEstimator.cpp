#include "StateEstimator.h"
#include <iostream>
StateEstimator::StateEstimator(std::map<int, LandmarkClass>& lm):landmarks(lm){
    noise_odom = 0.02*Eigen::Matrix3d::Identity(3,3);
    noise_odom(8)=0.1;
    noise_process = 0.05*Eigen::Matrix3d::Identity(3,3);
    noise_obs = 0.05*Eigen::Matrix3d::Identity(3,3);

    current_state = Eigen::Vector3d::Zero();
    current_cov = Eigen::Matrix3d::Identity(3,3);
    state_lock=false;
}

StateEstimator::~StateEstimator(){    
}

void StateEstimator::setStart(Eigen::Vector3d initState){
    state_lock = true;
    
    this->current_state = initState;
    current_cov = Eigen::Matrix3d::Identity(3,3);
    
    noise_odom = 0.02*Eigen::Matrix3d::Identity(3,3);
    noise_odom(8)=0.1;
    noise_process = 0.05*Eigen::Matrix3d::Identity(3,3);
    noise_obs = 0.1*Eigen::Matrix3d::Identity(3,3);

    state_lock=false;
}


Eigen::Vector3d StateEstimator::propagate(Eigen::Vector3d dpos){
    while(state_lock){
    }


    state_lock=true;

    //update covariance
    //Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3,3);
    //A(0,2) = -dpos(0)*std::sin(dpos(1));
    //A(1,2)= dpos(0)*std::cos(dpos(1));

    Eigen::MatrixXd B(3,3);
    B <<  std::cos(current_state(2)), -std::sin(current_state(2)), 0,
         std::sin(current_state(2)), std::cos(current_state(2)), 0,
            0,0,1;
    //std::cout<<B<<std::endl;
    // update states
    current_state = current_state + B*dpos;
    if(current_state(2)>M_PI){
        current_state(2) = current_state(2)-2*M_PI;
    }else if(current_state(2)<-M_PI){
        current_state(2) = current_state(2) +2*M_PI;
    }
    //current_cov = A*current_cov*A.transpose()+B*noise_obs*B.transpose();
    current_cov = current_cov+B*noise_odom*B.transpose()+noise_process;
    state_lock=false;
    return current_state;
}


Eigen::Vector3d predObservation(Eigen::VectorXd landmark, Eigen::Vector3d state){
    Eigen::Vector2d diff;
    diff << landmark(0) - state(0),
         landmark(1) - state(1);
    Eigen::Vector3d obs_prd;
    obs_prd << std::cos(state(2))*diff(0) + std::sin(state(2))*diff(1),
             -std::sin(state(2))*diff(0) + std::cos(state(2))*diff(1),
             landmark(2)-state(2);


    return obs_prd;
}

Eigen::Vector3d StateEstimator::tagDetectsCallback(std::vector<AprilTags::TagDetection> tagdetects){
    for(int i=0;i<tagdetects.size();i++){
        Eigen::Vector4d obs;
        obs<<tagdetects[i].id, tagdetects[i].hxy.first, tagdetects[i].hxy.second, tagdetects[i].observedPerimeter;
        update(obs);
    }
    return current_state;
}

void StateEstimator::update(Eigen::Vector4d obs){
    while(state_lock){
    }
    state_lock=true;

    for(std::map<int, LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        if (std::abs(obs(0)-it->first)<0.01){
            isam::Pose2d lm = it->second.isam_node->value();
            Eigen::Vector3d obs_pred = predObservation(lm.vector(),current_state);
            double c = std::cos(current_state(2)), s=std::sin(current_state(2));
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3,3);
            H <<  -c,  -s, obs_pred(1),
                   s,  -c, -obs_pred(0),
                    0,  0,  -1;

            //Eigen::MatrixXd R = 0.4*landmarks_[i](4)*Eigen::MatrixXd::Identity(2,2) + noise_obs;
            Eigen::MatrixXd R = noise_obs;
            Eigen::MatrixXd KalmanGain = (current_cov*H.transpose())*(H*current_cov*H.transpose()+R).inverse();
            current_state = current_state + KalmanGain*(obs.tail(3)-obs_pred);
            current_cov = current_cov - KalmanGain*H*current_cov;            
        }
    }
    state_lock = false;
}
