#pragma once
#include "common.h"
#include <fstream>
#include "Eigen/Dense"
#include "tools.h"
#include "debugging.h"
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
//#include <custom_msgs/GlobalMap.h>
#include <geometry_msgs/PoseArray.h>
#include <chrono>

#include<Eigen/StdVector>

class FastSlam {

static std::random_device rd_;
static std::mt19937 rand_engine_;

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

  //filter imu bias
  void InitializeIMU(sensor_msgs::ImuConstPtr state);

  // not currently used. Was used when for SLAM when running on AMZ simulation i think
  void load_map(Particle &p);
  // loads map from file, if we already know the track.
  void loadMapFromFile(Particle &p);

  /**
  *  Prediction for live execution
  */
  //void Prediction(sensor_msgs::ImuConstPtr state, sensor_msgs::ImuConstPtr last_state, boost::shared_ptr<const nav_msgs::Odometry> ins_vel);

  void Prediction2(const nav_msgs::Odometry* state, const nav_msgs::Odometry* last_state, boost::shared_ptr<const nav_msgs::Odometry> ins_vel);

  /**
  *  Correction for live execution
  */
  void Correction(const custom_msgs::ConeArrayConstPtr& temp_cloud);
  /**
  * Run the measurement model for each particle
  **/
  void Measurement_model(const Particle& p, const int& id, Eigen::Vector2d& h, Eigen::Matrix2d& H, Eigen::Matrix<double,2,3>& poseJacobian);

 /**
 * Run the resampling according to the weights for each particle
 **/

  void Resample();
  void LVResample();

  /**
  * Get Best Particle and its pose and landmark estimation
  Erik: not used anymore
  **/

  //void getBestPoseAndLandmark(VectorXd& mu);
  /**
  * Do data association for an observation given a particle
  **/
  //Data association just like in fastslam 1
  void Associate_observation(Particle &p, DataAssociation &da);

  void AdjustProposalDist(Particle &p, const RadarReading &obs);

  /**
  * updates landmark's position mean and cov in case it gets associated to an observation
  **/
  void update_landmark(Particle& p, int type, const DataAssociation &da);
  /**
  * Adds new landmark if observation couldn't be associated to an existing landmark
  **/
  void add_landmark(Particle& p, int type, const DataAssociation &da);


  /**
  * Functions that alter the map
  **/
  void decrease_landmark_counter(Particle &p);
  void remove_landmark(Particle &p);
  void CleanMap(Particle &p);

  void initialize_location(Particle &p, const custom_msgs::ConeArrayConstPtr &temp_cloud) const;
  bool isConeBelowThreshold(const LandMark &landmark);
  int MaxIndex();
  void multivariate_normal(Eigen::MatrixXd &sampledPose ,Eigen::Vector3d &mean,Eigen::Matrix3d &covariance,int samplesize=1);

  //================================================


public:
  // check whether the tracking toolbox was initialized or not (first measurement). Is set to true in the end of correction step.
  bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools_;
  // debug object used to log different values
  Debugging debug_;

  // number of particles
  static constexpr int N_ = 1; //240
  double Neff_limit_ = 1* N_; //if effective number of particles falls below Neff_limit_ resampling is done

  // a list of particles
  vector<Particle, Eigen::aligned_allocator<Particle>> particles_;

  // weights for each particle
  Eigen::Matrix<double, N_, 1> eigen_weights_;
  Eigen::Matrix<double, N_, 1> indexTable_;

  // motion noise
  Eigen::Matrix3d state_uncertainty_;
  Eigen::Matrix3d motion_noise_3D_ ;

  // measurement noise
  Eigen::Matrix2d Q_;

  // cut off value for new landmarks
  double p_new_landmark_;
  double punish_weight_;

  // max distance between mu and the observation
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


  bool amz_sim_;    //boolean ; true if running fastslam in amz simulation
  bool loc_mode_;
  bool setting_gps_;
  bool lvResample_=true; //true if Low Variance Resampling should be used, false for normal Resampling

  bool mapped_ = false;
  // ros time
  ros::Time last_cones_time;
  
  // lap counter
  int lap_=0;
  std::chrono::steady_clock::time_point lap_counter_pause_;
  std::chrono::steady_clock::time_point loop_closure_timer_start_;

  bool found_loop_closure_ = false;
  float wait_time_after_closure_;

  double last_pred_time_;

  double cone_covariance_threshold_ = 100.0;
  float min_cone_covariance_;
  float cones_minimum_dist_;
  int cone_counter_;
  bool map_cleaning_;

  int resamplingCounter=0;
  int effectiveResamplingCounter=0;
  int iteration_ =0;
};
