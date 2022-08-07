#include "planner.hpp"
#include <stdlib.h>
#include <fstream>

double squaredDist(double x1,double y1,double x2,double y2){
    return (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
}

double point_line_distance(double vx,double vy, double wx, double wy, double px,double py) {
   const float l2 = squaredDist(vx,vy,wx,wy);  // i.e. |w-v|^2 -  avoid a sqrt
   //if (l2 == 0.0) return distance(p, v);   // v == w case

   // Consider the line extending the segment, parameterized as v + t (w - v).
   // We find projection of point p onto the line.
   // It falls where t = [(p-v) . (w-v)] / |w-v|^2
   double t = ( (px-vx)*(wx-vx)+ (py-vy)*(wy-vy) ) / l2;

   if (t < 0.0){
	return std::sqrt( squaredDist(px,py,vx,vy) );       // Beyond the 'v' end of the segment
   }else if (t > 1.0){
	return std::sqrt( squaredDist(px,py,wx,wy) );  // Beyond the 'w' end of the segment
   }else{
	return std::sqrt( squaredDist(px,py, vx+t*(wx-vx), vy+t*(wy-vy)));
   }
}

Planner::Planner(std::map<int, LandmarkClass>& lm):landmarks(lm){
    x_min=100;
    x_max=-100;
    y_min=100;
    y_max=-100;
    landmark_size = 0.3;

    no_samples = 500;
    camera_view_distance = 5.0;

    goal_reach_threshold = 0.1;

    unknown_landmark_density = 3;
}

bool Planner::goalReached(Eigen::Vector3d current_pose){
    Eigen::Vector2d goal = path_to_follow.back().vector();
    if( (current_pose.head(2)-goal).norm() < goal_reach_threshold){
        return true;
    }else{
        return false;
    }
}

int Planner::headRightAngle(Eigen::Vector3d current_pose){
    // for debugging only
    //return true;

    isam::Point2d first_waypoint = path_to_follow.front();
    double angle_desired = std::atan2(first_waypoint.y()-current_pose(1),first_waypoint.x()-current_pose(0));
    if(std::abs(angle_desired-current_pose(2))<0.1){
        return 0;
    }else if(angle_desired-current_pose(2)>0){
	return 1;        
    }else{
	return -1;
    }
}

void Planner::AddNewAnchorPose(isam::Pose2d_Node* newpos){
    if(pose_history.size()==0){
        pose_history.push_back(newpos);
        return;
    }

    isam::Pose2d delpos = newpos->value().ominus( pose_history.back()->value() );
    if (delpos.vector().norm()>1){
        pose_history.push_back(newpos);
    }
}

double Planner::exploreBonus(std::vector<LandmarkClass> lms, isam::Point2d pose){
    //compute viewing angles
    std::map<int,double> angles;
    for(std::vector<LandmarkClass>::iterator it=lms.begin();it!=lms.end();it++){
        double d_theta = std::atan2(it->isam_node->value().y()-pose.y(), it->isam_node->value().x()-pose.x());
        angles.insert(std::pair<int,double>(it->id,d_theta));
    }

    //sort landmarks by their viewing angles
    for(int i=0;i<lms.size();i++){
        for(int j=i+1;j<lms.size();j++){
            if( angles[lms[i].id]>angles[lms[j].id]){
                LandmarkClass lm_temp = lms[i];
                lms[i]=lms[j];
                lms[j]=lm_temp;
            }
        }
    }

    double bonus=0;
    double multiplier = unknown_landmark_density*5*std::log(100);
    for(int i=1;i<lms.size();i++){
        if(!lms[i-1].left_connected && !lms[i].right_connected)
            bonus+= (angles[lms[i].id] - angles[lms[i-1].id]) * multiplier;
    }
    if(!lms.back().left_connected && !lms[0].right_connected){
        bonus+= (2*isam::PI-angles[lms.back().id] + angles[lms[0].id]) * multiplier;
    }


    return bonus;
}

void Planner::generateNextGoal(isam::Pose2d startpose, Eigen::MatrixXd covariance){
    this->start_pose = startpose;
    /**
     * step one, compute map boudaries
     */
    for(std::map<int,LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
        isam::Pose2d lm = it->second.isam_node->value();
        if(lm.x()<x_min){x_min = lm.x();}
        if(lm.x()>x_max){x_max = lm.x();}
        if(lm.y()<y_min){y_min = lm.y();}
        if(lm.y()>y_max){y_max = lm.y();}
    }

    /**
     * step two, generate samples
     */
    std::vector<isam::Point2d> samples;
    for(int i=0;i<no_samples;i++){
        //generate a sample point in space
        bool valid_sample = false;
        isam::Point2d sample((x_max-x_min)*rand()/RAND_MAX+x_min,(y_max-y_min)*rand()/RAND_MAX+y_min);

        // sample valid only when it can be seen by at least one of history poses
        for(std::list<isam::Pose2d_Node*>::iterator pose_it = pose_history.begin();pose_it!=pose_history.end();pose_it++){
            isam::Pose2d p = (*pose_it)->value();
            if(checkVisibility(p,sample) &&
                    checkFarFromObstacle(sample.x(),sample.y()) ){
                valid_sample=true;
                break;
            }
        }

        if(valid_sample){
          samples.push_back(sample);
        }
    }

    /**
      * step three, compute information gain on each of the samples
      */
    std::cout<<"compute information gain on each sample:"<<std::endl;
    double maxIReduction = -100000;
    int goal_idx = -1;
    for(int i=0;i<samples.size();i++){
        std::vector<LandmarkClass> lm_visible;
        std::vector<int> lm_idx;
        // find visible landmarks and compute information matrix
        int no_visible=0;
        for(std::map<int,LandmarkClass>::iterator lm_it=landmarks.begin();lm_it!=landmarks.end();lm_it++){
            isam::Pose2d lm = lm_it->second.isam_node->value();
            if(checkVisibility(lm,samples[i])){
                lm_visible.push_back(lm_it->second);
                lm_idx.push_back(no_visible);
            }
            no_visible++;
        }
        no_visible = lm_idx.size();
 	if(no_visible<3){
	    continue;
        }

        Eigen::MatrixXd cov_i(no_visible*3,no_visible*3);
        Eigen::MatrixXd A(no_visible*3,no_visible*3);
        for(int k1=0;k1<no_visible;k1++){
            for(int k2=0;k2<no_visible;k2++){
                cov_i.block(k1*3,k2*3,3,3) = covariance.block(lm_idx[k1]*3,lm_idx[k2]*3,3,3);
                if(k1==k2){
                    A.block(k1*3,k2*3,3,3) = 10.0*(no_visible-1)/no_visible*Eigen::MatrixXd::Identity(3,3);
                }else{
                    A.block(k1*3,k2*3,3,3) = -10.0/no_visible*Eigen::MatrixXd::Identity(3,3);
                }
            }
        }
	
        //compute entroy reduction
        double entropy_i = std::log( (Eigen::MatrixXd::Identity(no_visible*3,no_visible*3)+cov_i*A).determinant());
	entropy_i += exploreBonus(lm_visible,samples[i]);
	if(entropy_i>maxIReduction){
            maxIReduction = entropy_i;
            goal_idx = i;
        }	
    }

    /**
      * step four, build cost matrix between nodes      *
    */
    std::cout<<"build cost adjacent matrix"<<std::endl;
    isam::Point2d start(startpose.x(),startpose.y());
    samples.push_back(start);
    int n_sample = samples.size();
    Eigen::MatrixXd Adaj(n_sample,n_sample);
    for(int i=0;i<n_sample;i++){
        Adaj(i,i)=0;
        double x1=samples[i].x(), y1=samples[i].y();
        for(int j=i+1;j<n_sample;j++){
            double x2=samples[j].x(), y2=samples[j].y();
            double sq_dist = squaredDist(x1,y1,x2,y2);
            if(  sq_dist < camera_view_distance*camera_view_distance/4 &&
              checkVisibility(x1,y1,x2,y2)){
                Adaj(i,j) = std::sqrt(sq_dist);
                Adaj(j,i) = Adaj(i,j);
            }else{
                Adaj(i,j) = 1000;
                Adaj(j,i) = Adaj(i,j);
            }
        }
    }

    /**
      * step five, find shortest path
    */
    std::cout<<"extract shortest path"<<std::endl;
    Eigen::VectorXd dist_start=Adaj.col(n_sample-1);
    if( dist_start.minCoeff()>100){
	double x1=samples[n_sample-1].x(), y1=samples[n_sample-1].y();
	for(int j=0;j<n_sample-1;j++){
	    double x2=samples[j].x(), y2=samples[j].y();
	    double dist=std::sqrt(squaredDist(x1,y1,x2,y2));
	    if(dist < 0.5) dist_start[j]=dist;
	}        
    }
    int parent[n_sample];
    bool visited[n_sample];

    for(int i=0;i<n_sample-1;i++){
        visited[i]=false;
	if(dist_start(i)<1000){
	    parent[i]=n_sample-1;
        }else{
	    parent[i]=-2;
	}
    }
    parent[n_sample-1]=-1;
    visited[n_sample-1]=true;

    int idx = -1;
    //for(int k=0; k<n_sample-1;k++){
    while(idx!=goal_idx){
        // find shortest unvisited node
        double shortest = 1000;
        idx = -1;
        for(int j=0;j<n_sample;j++){
            if(!visited[j] && dist_start[j]<shortest){
                shortest = dist_start[j];
                idx =j;
            }
        }
	
	if(idx<0){
	    std::cout<<"dijikstra short path error"<<std::endl;
	}else{
	    //std::cout<<" add("<<idx<<","<<parent[idx]<<")";
	}
        // pivot distance for unvisited nodes
        for(int j=0;j<n_sample;j++){
            if( dist_start[j]>dist_start[idx]+Adaj(idx,j)){
                dist_start[j] = dist_start[idx]+Adaj(idx,j);
                parent[j] = idx;
            }
        }

        //mark idx as visited
        visited[idx]=true;      
    }

    std::cout<<"extract path from graph at "<<goal_idx<<std::endl;
    path_to_follow.clear();    
    while(parent[goal_idx]!=-1){
	if(goal_idx<0){
		std::cout<<"woops, error in computing shortest path"<<std::endl;
	}
        path_to_follow.push_front(samples[goal_idx]);
        goal_idx = parent[goal_idx];
    }
    
    /**
      * logger for debug    *
    */    
    /* std::ofstream log;
    log.open("/home/beipeng/Desktop/log.txt");
    
    for(std::map<int, LandmarkClass>::iterator it=landmarks.begin();it!=landmarks.end();it++){
	log<<it->first<<" "<<it->second.left_connected<<" "<<it->second.right_connected<<" "<<it->second.isam_node->value().vector().transpose()<<std::endl;
    }
    for(int i=0;i<samples.size();i++)
	log<<samples[i].x()<<" "<<samples[i].y()<<"\n";
    //for(int k=0;k<no_visible;k++) log<<" "<<lm_visible[k].id;
    
    for(int i=0;i<n_sample;i++){
        for(int j=i+1;j<n_sample;j++){
	    log<<" "<<i<<" "<<j<<"\n";
	}
    }                
    for(int k=0;k<n_sample;k++){
	log<<" "<<parent[k]<<" "<<dist_start[k];
    }
    log<<"\n";
    log.close();
    */
}

bool Planner::checkPathValidity(){
    if(path_to_follow.size()<2)
	return true;

    std::list<isam::Point2d>::iterator it_pre = path_to_follow.begin();
    for(std::list<isam::Point2d>::iterator it = (it_pre++); it!=path_to_follow.end();it++){
	if(!checkVisibility(it_pre->x(),it_pre->y(),it->x(),it->y() )){
	    return false;
	}
	it_pre = it;	
    }
    return true;    
}

bool Planner::checkFarFromObstacle(double x1, double y1){
    for(std::map<int,LandmarkClass>::iterator lm_it=landmarks.begin();lm_it!=landmarks.end();lm_it++){
        LandmarkClass lm1 = lm_it->second;
        for(std::map<int,int>::iterator nb_it= lm1.neighbors.begin();nb_it!=lm1.neighbors.end();nb_it++){
            LandmarkClass lm2 = landmarks[nb_it->first];
            double obs_x1 = lm1.isam_node->value().x(),
                   obs_y1 = lm1.isam_node->value().y(),
                   obs_x2 = lm2.isam_node->value().x(),
                   obs_y2 = lm2.isam_node->value().y();
            double dist=point_line_distance(obs_x1,obs_y1,obs_x2,obs_y2,x1,y1);
            if(dist>0 && dist<landmark_size){
                return false;
            }
        }
    }
    return true;
}

bool Planner::checkVisibility(double x1,double y1,double x2,double y2){
    // distance within field of view
    if( squaredDist(x1,y1,x2,y2)>camera_view_distance*camera_view_distance){
        return false;
    }

    // not blocked by any obsctacles
    for(std::map<int,LandmarkClass>::iterator lm_it=landmarks.begin();lm_it!=landmarks.end();lm_it++){
        // if landmark too close to the sight, it is blocking the view
        LandmarkClass lm1 = lm_it->second;
        double obs_x1 = lm1.isam_node->value().x(),
               obs_y1 = lm1.isam_node->value().y();
        double dist=point_line_distance(x1,y1,x2,y2,obs_x1,obs_y1);
        if(dist>0 && dist<landmark_size){
            return false;
        }

        for(std::map<int,int>::iterator nb_it= lm1.neighbors.begin();nb_it!=lm1.neighbors.end();nb_it++){
            LandmarkClass lm2 = landmarks[nb_it->first];
            double obs_x2 = lm2.isam_node->value().x(),
                   obs_y2 = lm2.isam_node->value().y();
            double s1_x, s1_y, s2_x, s2_y;
                s1_x = x2 - x1;     s1_y = y2 - y1;
                s2_x = obs_x2 - obs_x1;     s2_y = obs_y2 - obs_y1;

            double s, t;
                s = (-s1_y * (x1 - obs_x1) + s1_x * (y1 - obs_y1)) / (-s2_x * s1_y + s1_x * s2_y);
                t = ( s2_x * (y1 - obs_y1) - s2_y * (x1 - obs_x1)) / (-s2_x * s1_y + s1_x * s2_y);
            if (s > 0 && s < 1 && t > 0 && t < 1){
                return false;
            }
        }
    }
    return true;
}

bool Planner::checkVisibility(isam::Pose2d p1, isam::Point2d p2){
    double x1=p1.x(),y1=p1.y(),x2=p2.x(),y2=p2.y();

    //angle within field of view
    double angle = std::atan2(y2-y1,x2-x1) - p1.t();
    if(angle>isam::PI) angle=angle-2*isam::PI;
    if(angle<-isam::PI) angle = angle + 2*isam::PI;
    if( angle<isam::PI/2 && angle>-isam::PI/2){
        return false;
    }

    return checkVisibility(x1,y1,x2,y2);
}
