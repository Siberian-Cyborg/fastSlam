#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "common.h"
// can be removed in Eigen 3.3.9
#include <Eigen/StdVector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;



struct LandMark {
  int id;
  int counter = 0;
  Eigen::Vector2d mu;
  Eigen::Matrix2d sigma;
  bool observed;
  std::vector<double>color_probabilities{1.0/3.0,1.0/3.0,1.0/3.0};
  int alpha_r = 1;
  int alpha_l = 1;
  int alpha_u = 1;
  int type = 3; //0->left , 1->right, 2->orange? 3->unknown
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct DataAssociation{
  LandMark landmark;
  RadarReading reading;
  double probability;
  Eigen::Matrix2d InnovationMatrix;
  Eigen::Matrix2d FeatureJacobian;
  Eigen::Matrix<double,2,3> PoseJacobian;
  Eigen::Vector2d Measurement_diff;
};

struct Particle {
  int id;
  int parent_id;
  Eigen::Vector3d pose;
  Eigen::Vector3d sampledPose;
  Eigen::Vector3d vel;
  Eigen::Vector3d mu;
  Eigen::Matrix3d sigma;
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> history;
  vector<LandMark,Eigen::aligned_allocator<LandMark>> landmarks;
  double weight;
  vector<DataAssociation, Eigen::aligned_allocator<DataAssociation>> data_associations;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * Noarmlize the angle
  */
  float normalize_angle(float phi);
  
  /**
  * Normalized the angles from observation
  */
  void normalize_bearing(VectorXd& Z);

  double gaussianXd(VectorXd& x, MatrixXd& covariance);


};

#endif /* TOOLS_H_ */
