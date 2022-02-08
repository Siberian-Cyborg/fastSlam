#ifndef DEBUGGING_H_
#define DEBUGGING_H_
#include "Eigen/Dense"
#include "tools.h"

class Debugging{

public:
  /**
  * Constructor.
  */
    Debugging();

  /**
  * Destructor.
  */
    virtual ~Debugging();

    void safeParticleWeights(int num);


    void testMultivariate(Eigen::MatrixXd &sample, Eigen::Vector3d &mean, Eigen::Matrix3d &cov, Eigen::MatrixXd &randM, Eigen::Matrix3d &normTrans, bool chol);

    void safeParticle(int num);

    void checkDataAssociations();

    void checkValues(const Eigen::Matrix2d &Z, const Eigen::Matrix2d &myFeatureJacobian,
                           const Eigen::Matrix<double,2,3> &myPoseJacobian, const Eigen::Matrix2d &l_sig,
                           const Eigen::Matrix3d &p_sig, const Eigen::Vector3d &p_mu, const double &pr,
                           const Eigen::Vector3d &s_pose, const Eigen::Vector2d &n_z_diff, const Eigen::Vector2d &z_dif,
                           const Eigen::Matrix2d &Z_in, const Eigen::Matrix3d &noises_in, const Eigen::Matrix3d &temporal, const Eigen::Matrix3d &noises);

    void checkSampledParticle(const Eigen::Vector3d &sampled_pose,const Particle &p,const Eigen::Vector2d &z_diff_before,const Eigen::Vector2d &z_diff_after, const double &proba);

};

#endif /* DEBUGGING_H_ */