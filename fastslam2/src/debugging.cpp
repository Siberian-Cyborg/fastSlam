#include <iostream>
#include "debugging.h"
#include "fastslam.h"



FastSlam fastslam;

Debugging::Debugging() {}

Debugging::~Debugging() {}

//For debugging. Writes newly sampled particle into txt document to see if the sampling process makes sense.
void Debugging::checkSampledParticle(const Eigen::Vector3d &sampled_pose, const Particle &p, const Eigen::Vector2d &z_diff_before, const Eigen::Vector2d &z_diff_after, const double &proba){

    std::string homepath = getenv("HOME");
    std::ofstream myfile;
    myfile.open ((homepath + "/CheckSampledParticle.txt").c_str(),std::ios_base::app);
    myfile << "particle id: "<<std::setw(10)<<p.id<<"\n";
    myfile <<"pose\n";
    myfile<<p.pose<<"\n";
    myfile <<"mu\n";
    myfile<<p.mu<<"\n";
    myfile <<"sigma\n";
    myfile<<p.sigma<<"\n";
    myfile <<"sampled pose\n";
    myfile<<sampled_pose<<"\n";
    myfile <<"z_diff before sampling\n";
    myfile<<z_diff_before<<"\n";
    myfile <<"z_diff after sampling\n";
    myfile<<z_diff_after<<"\n";
    myfile <<"probability of association\n";
    myfile<<proba<<"\n";
    myfile <<"________________________\n";
    myfile.close();
}


void Debugging::checkValues(const Eigen::Matrix2d &Z, const Eigen::Matrix2d &myFeatureJacobian,
                           const Eigen::Matrix<double,2,3> &myPoseJacobian, const Eigen::Matrix2d &l_sig,
                           const Eigen::Matrix3d &p_sig, const Eigen::Vector3d &p_mu, const double &pr,
                           const Eigen::Vector3d &s_pose, const Eigen::Vector2d &n_z_diff, const Eigen::Vector2d &z_dif,
                           const Eigen::Matrix2d &Z_in, const Eigen::Matrix3d &noises_in, const Eigen::Matrix3d &temporal, const Eigen::Matrix3d &noises){

    std::string homepath = getenv("HOME");
    std::ofstream myfile;
    myfile.open((homepath + "/Matrices.txt").c_str(), std::ios_base::app);

    myfile << "Z Matrix\n";
    myfile << Z << "\n";
    myfile << "Z Matrix inverse\n";
    myfile << Z_in << "\n";
    myfile << "Noises Matrix\n";
    myfile << noises << "\n";
    myfile << "Noises Matrix inverse\n";
    myfile << noises_in << "\n";
    myfile << "Feature Jacobian\n";
    myfile << myFeatureJacobian << "\n";
    myfile << "Pose Jacobian\n";
    myfile << myPoseJacobian << "\n";
    myfile << "Landmark Covariance\n";
    myfile << l_sig << "\n";
    myfile << "temporal\n";
    myfile << temporal << "\n";
    myfile << "Particle Covariance\n";
    myfile << p_sig << "\n";
    myfile << "Particle Mean Position\n";
    myfile << p_mu << "\n";
    myfile << "Probability of association\n";
    myfile << pr << "\n";
    myfile << "Observation difference before sampling\n";
    myfile << z_dif << "\n";
    myfile << "Sampled pose\n";
    myfile << s_pose << "\n";
    myfile << "Observation difference after sampling\n";
    myfile << n_z_diff << "\n";
    myfile << "________________________\n";

    myfile.close();
}


void Debugging::checkDataAssociations(){
    std::string homepath = getenv("HOME");
    std::ofstream myfile;
    myfile.open((homepath + "/DataAssociations.txt").c_str(), std::ios_base::app);
    for(auto &p : fastslam.particles_){
        myfile << "particle id: " << std::setw(10) << p.id << "\n";

        for(auto &da : p.data_associations){
            myfile << "probability of data association\n";
            myfile << da.probability << "\n";
            myfile << "id of associated landmark\n";
            myfile << da.landmark.id << "\n";
            myfile << "id of respective radar reading\n";
            myfile << da.reading.id << "\n";
            myfile << "________________________\n";
        }

        myfile <<"=============================================\n";
    }
    myfile.close();
}

void Debugging::safeParticle(int num){
   std::string homepath = getenv("HOME");
   std::ofstream myfile;
   if(num==0){
        myfile.open ((homepath + "/particlePrediction.txt").c_str(),std::ios_base::app);
    }
    else if(num==1){
        myfile.open ((homepath + "/particleCorrection.txt").c_str(),std::ios_base::app);
    }
    else if(num==2){
        myfile.open ((homepath + "/particleResampling.txt").c_str(),std::ios_base::app);
    }
    for(auto &p : fastslam.particles_){
        myfile << "iteration: "<<std::setw(10)<<fastslam.iteration_<<"\n";
        myfile << "\n";
        myfile << "parent id: "<<std::setw(10)<<p.parent_id<<"\n";
        myfile << "particle id: "<<std::setw(10)<<p.id<<"\n";
        myfile <<"weight: "<<std::setw(10)<<p.weight<<"\n";
        myfile <<"pose\n";
        myfile<<p.pose<<"\n";
        myfile <<"________________________\n";
    }
   myfile <<"=============================================\n";
   fastslam.iteration_++;
   myfile.close();
}


//meant for testing and debugging the multivariate sampling function
void Debugging::testMultivariate(Eigen::MatrixXd &sample, Eigen::Vector3d &mean, Eigen::Matrix3d &cov, Eigen::MatrixXd &randM, Eigen::Matrix3d &normTrans,  bool chol){

   std::string homepath = getenv("HOME");
   std::ofstream myfile;
   myfile.open ((homepath + "/testMultivariate.txt").c_str(),std::ios_base::app);

   if(chol){
    myfile << "Calculated using cholesky \n";
   }else{
    myfile << "Calculated using eigenvalue decomposition \n";
   }

        myfile << "the sample result is: "<< sample.rows()<<" x "<<sample.cols()<<"\n";

        if(sample.cols()<10){
        myfile << "Random Matrix: \n";
        myfile <<randM<<"\n";
        myfile << "\n";
        }

        myfile << "Cholesky Decomposition L: \n";
        myfile <<normTrans<<"\n";
        myfile << "\n";

        myfile << "L*L.transpose(): \n";
        myfile <<normTrans*normTrans.transpose()<<"\n";
        myfile << "\n";

        myfile << "mean: \n";
        myfile <<mean<<"\n";

        myfile << "covariance: \n";
        myfile <<cov<<"\n";
        myfile << "\n";

        if(sample.cols()<10){
        myfile << "sampled result: \n";
        myfile <<sample<<"\n";
        myfile <<"________________________\n";
        }

    if(sample.cols()>=1000){ //computing the covariance and the mean only makes sense for a very big samplesize of around 1000
    myfile <<"calculated mean:\n";
    myfile <<sample.rowwise().mean()<<"\n";
    myfile << "\n";

    Eigen::MatrixXd centered = sample.colwise() - sample.rowwise().mean();
    Eigen::MatrixXd calc_cov = (centered * centered.transpose()) / double(sample.cols() - 1);

    myfile <<"calculated cov:\n";
    myfile <<calc_cov<<"\n";
    myfile << "\n";
    }

   myfile.close();
}


//meant for debugging
//creates txt files. Text files need to be deleted prior, else new info is only appended to existing file (doesnt override).
void Debugging::safeParticleWeights(int num){

    std::cout<<"entered Debug"<<std::endl;
    std::string homepath = getenv("HOME");
	std::ofstream myfile;
	if(fastslam.lvResample_){
	    myfile.open ((homepath + "/particle_weights_LVR.txt").c_str(),std::ios_base::app); //LVR is Low Variance resampling
	}else{
	    myfile.open ((homepath + "/particle_weights_NR.txt").c_str(),std::ios_base::app); // Normal Resampling
	}

  	if(num==0){
  	    myfile<<"Weights before Resampling\n";
  	    myfile<<"_________________________\n";
  	}
  	if(num==1){
  	    myfile<<"Weights after Low Variance Resampling \n";
  	    myfile<<"_________________________\n";
  	}
  	if(num==2){
  	    myfile<<"Weights after Normal Resampling\n";
  	    myfile<<"_________________________\n";
  	}


    if(num==0){
        for (int i = 0; i < fastslam.N_; i++){
            myfile << "particle id: "<<fastslam.particles_[i].id<<"\n";
            myfile <<fastslam.particles_[i].weight<<"\n";
        }
    }else{
        //calculateIndexTable();
        for (int i = 0; i < fastslam.N_; i++){
            if(i==0){
                //myfile <<std::setw(30)<<"particle id: "<<i<<"\n";
                myfile <<std::setw(30)<<"resample particle id: "<<fastslam.particles_[i].id<<" taken from id: "<<fastslam.particles_[i].parent_id<<"\n";
                myfile <<std::setw(30)<<fastslam.particles_[i].weight<<"\n";
            }else{
                if(fastslam.particles_[i-1].weight==fastslam.particles_[i].weight){
                    myfile <<std::setw(30)<<fastslam.particles_[i].weight<<"\n";
                }else{
                    myfile <<std::setw(30)<<"\n";
                    //myfile <<std::setw(30)<< "particle id: "<<i<<"\n";
                    myfile <<std::setw(30)<<"resample particle id: "<<fastslam.particles_[i].id<<" taken from id: "<<fastslam.particles_[i].parent_id<<"\n";
                    myfile <<std::setw(30)<<fastslam.particles_[i].weight<<"\n";

                }
            }
        }
    }
	myfile.close();
}