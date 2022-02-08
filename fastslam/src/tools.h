#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
// can be removed in Eigen 3.3.9
#include <Eigen/StdVector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


struct LandMark {
  int id;
  Eigen::Vector2d mu;
  Eigen::Matrix2d sigma;
  bool observed;
  std::vector<double>color_probabilities{1.0/3.0,1.0/3.0,1.0/3.0};
  int alpha_r = 1;
  int alpha_l = 1;
  int alpha_u = 1;
  int type = 2; //0->left , 1->right, 2->unknown
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Particle {
  int id;
  Eigen::Vector3d pose;
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> history;
  vector<LandMark,Eigen::aligned_allocator<LandMark>> landmarks;
  Eigen::Vector3d vel;
  double weight;
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
};

#endif /* TOOLS_H_ */
