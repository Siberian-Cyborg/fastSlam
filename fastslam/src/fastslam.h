#pragma once
#include "common.h"
#include <fstream>
#include "Eigen/Dense"
#include "tools.h"
#include "ros/ros.h"
#include <random>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <custom_msgs/WheelValues.h>
#include <custom_msgs/ConeArray.h>
#include <custom_msgs/Cone.h>
#include <custom_msgs/LogValue.h>
#include <custom_msgs/LogArray.h>
#include <geometry_msgs/PoseArray.h>
#include <chrono>

#include<Eigen/StdVector>

class FastSlam {
public:
  /**
  * Constructor, initialize with 100 partilces;
  */
  FastSlam();

  /**
  * Destructor.
  */
  virtual ~FastSlam();

  /**
  * Initialize the landmark size and number of particles
  */
  void Initialize(ros::NodeHandle nh);

  void loadMapFromFile(Particle &p);

  /**
  *  Prediction for live execution
  */
  //void Prediction(custom_msgs::WheelValuesConstPtr state, custom_msgs::WheelValuesConstPtr last_state, boost::shared_ptr<const nav_msgs::Odometry> ins_vel);
  void Prediction(const nav_msgs::Odometry* state,
                  const nav_msgs::Odometry* last_state,
                  boost::shared_ptr<const nav_msgs::Odometry> ins_vel);
  /**
  *  Correction for live execution
  */
  void Correction(const custom_msgs::ConeArrayConstPtr& temp_cloud);
  /**
  * Run the measurement model for each particle
  **/
  void Measurement_model(const Particle& p, const int& id, Eigen::Vector2d& h, Eigen::Matrix2d& H);

 /**
 * Run the resampling according to the weights for each particle
 **/

  void Resample();
  /**
  * Get Best Particle and its pose and landmark estimation
  **/

  void getBestPoseAndLandmark(VectorXd& mu);
  /**
  * Do data association for an observation given a particle
  **/
  
  void Associate_observation(const Particle& p,
                      const double& gx,
                      const double& gy,
                      Eigen::Matrix2d& best_sig, 
                      Eigen::Matrix2d& best_Q,
                      Eigen::Matrix2d& best_K,
                      Eigen::Matrix2d& best_H,
                      Eigen::Vector2d& best_z_diff,
                      const RadarReading& z,
                      double& nearest_distance,
                      double& best_prob,
                      int& l,
                      int type);
  
  /**
  * updates landmark's position mean and cov in case it gets associated to an observation
  **/
  void update_landmark(Particle& p, 
                      const Eigen::Matrix2d& best_Q, 
                      const Eigen::Matrix2d& best_H, 
                      const Eigen::Vector2d& best_z_diff,
                      const int& l,
                      const double& best_prob,
                      int type);
  /**
  * Adds new landmark if observation couldn't be associated to an existing landmark
  **/
  void add_landmark(Particle&p ,const double& gx, const double&  gy, RadarReading& z, int type);

  void load_map(Particle &p);
  void initialize_location(Particle &p, const custom_msgs::ConeArrayConstPtr &temp_cloud) const;
  bool isConeBelowThreshold(const LandMark &landmark);
  //================================================
  int MaxIndex();

public:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools_;

  // number of particles
  static constexpr int N_ = 60;

  // a list of particles
  vector<Particle, Eigen::aligned_allocator<Particle>> particles_;

  // weights for each particle
  vector<double> weights_;
  Eigen::Matrix<double, N_, 1> eigen_weights_;

  // noises for pose
  Eigen::Matrix2d noises_;

  // mesurement noise
  Eigen::Matrix2d Q_;

  // cut off value for new landmarks
  double p_new_landmark_;

  // max distance between mu and the obersvation
  double max_matching_distance_;

  // imu initialization
  Eigen::Vector3d imu_biases_;
  std::vector<sensor_msgs::ImuConstPtr> inertial_queue_;
  bool inertial_init_ = true;

  //initial coordinates (needed for lap counter in sim mode)
  double initial_x_;
  double initial_y_;

  // lidar offset
  double x_offset_;
  double y_offset_;
  double yaw_offset_;

  // max distance from the car to be considered a reasonable observation
  double max_considered_distance_;
  double min_considered_distance_;

  //boolean ; true if running fastslam in amz simulation
  bool sim_;
  bool loc_mode_;
  bool setting_gps_;
  bool mapped_ = false;

  // ros time
  ros::Time last_cones_time;
  
  // lap counter
  int lap_;
  std::chrono::steady_clock::time_point lap_counter_pause_;

  double last_pred_time_;

  double cone_covariance_threshold_ = 100.0;
};
