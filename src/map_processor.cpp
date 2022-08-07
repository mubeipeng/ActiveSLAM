#include "map_processor.hpp"
#include <string>

MapProcessor::MapProcessor(std::map<int, LandmarkClass>& lm):landmarks(lm){
    Eigen::MatrixXd m_odom = 10.*isam::eye(3);
    m_odom(2,2)=0.1;
    noise_odom = isam::Information(m_odom);
    noise_lm = isam::Information(m_odom);


    isam::Pose2d origin(0., 0., 0.);
    last_pose = new isam::Pose2d_Node();
    isam::Pose2d_Factor* prior = new isam::Pose2d_Factor(last_pose, origin, noise_odom);
      // add it to the graph
    slam.add_factor(prior);
    delpos.set(0,0,0);

    //viewer.addCoordinateSystem(5,0);
    //batch_every = 20;
    //step=0;
}

MapProcessor::~MapProcessor(){
    /*
    for(std::map<int,LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        isam::Pose2d lm = it->second.isam_node->value();
        std::cout<<it->first<<" "<<lm.x()<<" "<<lm.y()<<" "<<lm.t()<<std::endl;
    }
    for(std::map<int,LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        LandmarkClass lm = it->second;
        for(std::map<int,int>::iterator it_nb = lm.neighbors.begin();it_nb!=lm.neighbors.end();it_nb++)
        	std::cout<<it->first<<" "<<it_nb->first<<" "<<it_nb->second;
	std::cout<<std::endl;
    }*/
}

Eigen::Vector3d MapProcessor::updateOdom(isam::Pose2d odom_reading){
    delpos = delpos.oplus(odom_reading);
    Eigen::Vector3d current_pose = last_pose->value().oplus(delpos).vector();
    if(current_pose(2)>isam::PI){
        current_pose(2) = current_pose(2)-2*isam::PI;
    }else if(current_pose(2)<-isam::PI){
        current_pose(2) = current_pose(2)+2*isam::PI;
    }
    return current_pose;
}

isam::Pose2d_Node* MapProcessor::updateFeature(Eigen::MatrixXd laser, std::vector<AprilTags::TagDetection> m_tagDetections){

    isam::Pose2d_Node*  current_pose = new isam::Pose2d_Node();    
    isam::Pose2d_Pose2d_Factor* pos_factor = new isam::Pose2d_Pose2d_Factor(last_pose, current_pose, delpos, noise_odom);
    slam.add_node(current_pose);
    slam.add_factor(pos_factor);

    for(int i=0;i<m_tagDetections.size();i++){
        isam::Pose2d measure(m_tagDetections[i].hxy.first,m_tagDetections[i].hxy.second, m_tagDetections[i].observedPerimeter);
        //isam::Point2d measure(laser(tag_loc,0),laser(tag_loc,1));
        LandmarkClass landmark_i = landmarks[m_tagDetections[i].id];
        if(!landmark_i.isam_node){
            landmark_i.id = m_tagDetections[i].id;
            landmark_i.isam_node = new isam::Pose2d_Node();
            slam.add_node(landmark_i.isam_node);
            landmarks[m_tagDetections[i].id] = landmark_i;
        }
        //isam::Pose2d_Point2d_Factor* measure_factor = new isam::Pose2d_Point2d_Factor(current_pose,landmark_i.isam_node,measure,noise2);
        isam::Pose2d_Pose2d_Factor* measure_factor = new isam::Pose2d_Pose2d_Factor(current_pose,landmark_i.isam_node,measure,noise_lm);
        slam.add_factor(measure_factor);
    }

    updateTopology(laser,m_tagDetections);

    /*
    if(step==batch_every){
        slam.batch_optimization();
        step=0;
    }else{
        step++;
        //slam.update();
    }*/

    last_pose = current_pose;
    delpos.set(0,0,0);
    m_tagDetections.clear();

    return current_pose;
}

/*
std::vector<float> computeNormal(Eigen::MatrixXd laser){
    std::vector<float> normals;
    normals.resize(laser.rows());

    for(int i=0;i<laser.rows();i++){
        // step 1, find neighbor points
        std::list<Eigen::Vector2d> neighbors;
        for(int j=0;j<laser.rows();j++){
            float dist = (laser.row(i) - laser.row(j)).norm();
            if(laser(j,1)>0 && dist<0.15)
                neighbors.push_back(laser.row(j));
        }

        // step 2, check if there are enough neighbor points
        if(neighbors.size()<2){
            normals[i] = -1000;
            continue;
        }

        //step 3, compute normal
        Eigen::Vector2d mean;
        mean <<0,0;

        for(std::list<Eigen::Vector2d>::iterator it=neighbors.begin();it!=neighbors.end();it++){
            mean = mean+ *it;
        }
        mean = mean/neighbors.size();
        double num=0,den=0;
        for(std::list<Eigen::Vector2d>::iterator it=neighbors.begin();it!=neighbors.end();it++){
            Eigen::Vector2d n_i = *it;
            num += (n_i(0)-mean(0))*( n_i(1)-mean(1) );
            den += ( n_i(0)-mean(0) )*( n_i(0)-mean(0) );
        }
        normals[i] = num/den;
    }

    return normals;
}*/

std::vector<int> segment(Eigen::MatrixXd laser){
    // segement
    std::vector<int> component;
    component.resize(laser.rows());
    component[0]=1;
    int no_comp = 1;
    for(int i=1;i<laser.rows();i++){
        component[i]=no_comp+1;

        // search i-30:i points for neighbors
        int j=i-30; if(j<0) j=0;
        while( j<i ){
            float dist = (laser.row(i)-laser.row(j)).norm();
            if(dist<0.2 && component[j]<component[i]){
                component[i]=component[j];
            }
            j++;
        }
        if(component[i]>no_comp){
            no_comp++;
        }
    }
    return component;
}

void MapProcessor::updateTopology(Eigen::MatrixXd laser,std::vector<AprilTags::TagDetection> m_tagDetections){
    std::vector<int> component = segment(laser);

    // sort tag detections from right to left
    for(int i=0;i<m_tagDetections.size();i++){
        for(int j=i+1;j<m_tagDetections.size();j++){
            if(m_tagDetections[i].cxy.first<m_tagDetections[j].cxy.first){
                AprilTags::TagDetection tagdetect=m_tagDetections[i];
                m_tagDetections[i]=m_tagDetections[j];
                m_tagDetections[j]=tagdetect;
            }
        }
    }

    //create neighbors
    for(int i=1;i<m_tagDetections.size();i++){

        // update wall length
        //int center,left,right;
        //center = m_tagDetections[i].cxy.first;
        //left = center;
        //while(left>0 && std::abs(normal[left]-normal[center])<0.4
        //      && component[left]==component[center]){
        //    left--;
        //}
        //left++;
        //float left_dist = ( laser.row(left)-laser.row(center)).norm();
        //if(landmarks[m_tagDetections[i].id].left_size<left_dist){
        //   landmarks[m_tagDetections[i].id].left_size=left_dist;
        //}
        //right = center;
        //while(right<laser.rows()-1 && std::abs(normal[right]-normal[center])<0.4
        //      && component[center]==component[right]){
        //    right++;
        //}
        //right--;
        //float right_dist = ( laser.row(right)-laser.row(center)).norm();
        //if(landmarks[m_tagDetections[i].id].right_size<right_dist){
        //   landmarks[m_tagDetections[i].id].right_size=right_dist;
        //}

        //update neighbors
        AprilTags::TagDetection tag_l = m_tagDetections[i], tag_r=m_tagDetections[i-1];
	LandmarkClass lm_l = landmarks[tag_l.id], lm_r = landmarks[tag_r.id];
	Eigen::Vector3d dpos = lm_l.isam_node->value().ominus(lm_r.isam_node->value()).vector();
	double sqr_dist = dpos(0)*dpos(0)+dpos(1)*dpos(1);
	if ( component[tag_l.cxy.first]==component[tag_r.cxy.first] && sqr_dist< 2.0*2.0 ){
            if(lm_l.neighbors[tag_r.id]){
                landmarks[tag_l.id].neighbors[tag_r.id]++;
                landmarks[tag_r.id].neighbors[tag_l.id]++;
            }else{
                landmarks[tag_l.id].neighbors.insert(std::pair<int,int>(tag_r.id,1));
                landmarks[tag_r.id].neighbors.insert(std::pair<int,int>(tag_l.id,1));
            }
	    landmarks[tag_r.id].left_connected = true;
            landmarks[tag_l.id].right_connected = true;
        }
    }
}

 void MapProcessor::updateMap(isam::Pose2d& currentp,Eigen::MatrixXd& covariance){
    slam.batch_optimization();
    std::list<isam::Node*> lms;
    for(std::map<int,LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        lms.push_back(it->second.isam_node);
    }
    covariance = slam.covariances().marginal(lms);
    currentp = last_pose->value();
}


