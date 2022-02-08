#include "fastslam.h"
#include "profc.h"
#include <omp.h>
/*
 * Constructor.
 */
FastSlam::FastSlam() { is_initialized_ = false; }

const double rear_axle_offset = 0.6;

/**
Initialize the parameters
**/
void FastSlam::Initialize(ros::NodeHandle nh)
{
	std::map<std::string, double> noise_map, Q_map, offset_map;
	nh.getParam("noise", noise_map);
	nh.getParam("Q", Q_map);
	nh.getParam("offset", offset_map);
	nh.getParam("sim", sim_);
	nh.getParam("loc_mode", loc_mode_);
	setting_gps_ = true;
	nh.getParam("gps_fusion", setting_gps_);
	nh.getParam("cone_covariance_threshold", cone_covariance_threshold_);

	if (setting_gps_) 
	{
		ROS_WARN_STREAM("Using GPS fusion!");
	} else {
		ROS_WARN_STREAM("GPS fusion Disabled!");
	}

	noises_ = Eigen::Matrix2d::Zero();
	noises_(0, 0) = noise_map["r1"];
	noises_(0, 1) = noise_map["r2"];
	noises_(1, 0) = noise_map["tlong"];
	noises_(1, 1) = noise_map["tlat"];

	Q_ = Eigen::Matrix2d::Zero();
	Q_(0, 0) = Q_map["_00"];
	Q_(0, 1) = Q_map["_01"];
	Q_(1, 0) = Q_map["_10"];
	Q_(1, 1) = Q_map["_11"];

	x_offset_ = y_offset_ = yaw_offset_ = 0.0;
	x_offset_ = offset_map["x"];
	y_offset_ = offset_map["y"];
	yaw_offset_ = offset_map["yaw"];

	if (!nh.param<double>("p_new_landmark", p_new_landmark_, 0.9))
	{
		ROS_WARN_STREAM("Did not load p_new_landmark. Standard value is: " << p_new_landmark_);
	}
	
	if (!nh.param<double>("max_matching_distance", max_matching_distance_, 3.0))
	{
		ROS_WARN_STREAM(
				"Did not load max_matching_distance. Standard value is: "
				<< max_matching_distance_);
	}
	if (!nh.param<double>("max_considered_distance", max_considered_distance_, 10.0))
	{
		ROS_WARN_STREAM(
				"Did not load max_considered_distance. Standard value is: "
				<< max_considered_distance_);
	}
	if (!nh.param<double>("min_considered_distance", min_considered_distance_, 2.0))
	{
		ROS_WARN_STREAM(
				"Did not load min_considered_distance. Standard value is: "
				<< min_considered_distance_);
	}
	// if relocalizing, load map
	if (!loc_mode_)
	{
		// not relocalizing...
		particles_.resize(N_);
		weights_.resize(N_);
		for (int i = 0; i < N_; i++)
		{
			particles_[i].weight = 1.0 / N_;
			particles_[i].pose << 0, 0, 0;
			particles_[i].vel << 0, 0, 0;
			particles_[i].landmarks = vector<LandMark,Eigen::aligned_allocator<LandMark>>();

		}
	}
	else
	{
		particles_.resize(N_);
		weights_.resize(N_);
		for (int i = 0; i < N_; i++)
		{
			particles_[i].weight = 1.0 / N_;
			particles_[i].pose << 0, 0, 0;
			load_map(particles_[i]);
		}
	}
}
void FastSlam::load_map(Particle &p)
{
	std::string homepath = getenv("HOME");
	std::ifstream file((homepath + "/map_live.txt").c_str());
	std::string str;
	double x, y;
	int counter = 0;
	int type;
	if (file.is_open())
	{
		while (std::getline(file, str))
		{
			if (str[0] == 'i')
			{
				std::getline(file, str);
				std::istringstream ss_x(str);
				ss_x >> x;

				std::getline(file, str);
				std::istringstream ss_y(str);
				ss_y >> y;

				if (!sim_)
				{
					std::getline(file, str);
					std::istringstream ss_type(str);
					ss_type >> type;
				}

				LandMark newLandmark;
				newLandmark.id = counter;
				counter++;
				newLandmark.mu << x, y;
				newLandmark.sigma << 0.001, 0.001, 0.001, 0.001;
				newLandmark.type = type;
				p.landmarks.push_back(newLandmark);
			}
		}
	}
	file.close();
}
/**
 * Destructor.
 */
FastSlam::~FastSlam() {}


void FastSlam::loadMapFromFile(Particle &p)
{
	std::string homepath = getenv("HOME");
	std::ifstream file((homepath + "/db020/modules_linux/03_stateestimation/map.txt").c_str());
	std::string str;
	double x, y;
	int counter = 0;
	if (file.is_open())
	{
	    ROS_WARN_STREAM("Map is being loaded from file!");
	    mapped_ = true;
		while (std::getline(file, str))
		{
			if (str[0] == 'i')
			{
				std::getline(file, str);
				std::istringstream ss_x(str);
				ss_x >> x;

				std::getline(file, str);
				std::istringstream ss_y(str);
				ss_y >> y;

				LandMark newLandmark;
				newLandmark.id = counter;
				counter++;
				newLandmark.mu << x, y;
				newLandmark.sigma << 0.001, 0.001, 0.001, 0.001;
				p.landmarks.push_back(newLandmark);
			}
		}
	}
	file.close();
}

/**
 * measurement_model
 * for current pose and landmark
 * p - particle
 * id - which landmark
 * h - measurement - output
 * H - Jacobian - output
 */
void FastSlam::Measurement_model(const Particle &p, const int &id, Eigen::Vector2d &h, Eigen::Matrix2d &H)
{
	// int id = z.id;
	auto pose = p.landmarks[id].mu;
	// use the current state of particle to predict measurement
	double delta_x = pose(0) - p.pose(0);
	double delta_y = pose(1) - p.pose(1);
	// bearing and range to selected cone
	double expect_range = sqrt(delta_x * delta_x + delta_y * delta_y);
	double expect_bearing =
			tools_.normalize_angle(atan2(delta_y, delta_x) - p.pose(2));
	h << expect_range, expect_bearing;
	// compute the Jacobian H of the measurement function h wrt to the landmark
	// location
	// TODO(Stan): refactor
	H = Eigen::Matrix2d::Zero();
	H(0, 0) = delta_x / expect_range;
	H(0, 1) = delta_y / expect_range;
	H(1, 0) = -delta_y / (expect_range * expect_range);
	H(1, 1) = delta_x / (expect_range * expect_range);
}
//prediction for live execution
void FastSlam::Prediction(const nav_msgs::Odometry* state,
                          const nav_msgs::Odometry* last_state,
                          boost::shared_ptr<const nav_msgs::Odometry> ins_vel)
{
	PROFC_NODE("prediction");

    last_pred_time_ = state->header.stamp.toSec();

	if ( ins_vel == nullptr ) {
//		std::cout << "No GPS" << std::endl;
	}

	if (last_state == nullptr)
	{
		for (auto &p : particles_)
		{
			p.pose(0) = 0;
			p.pose(1) = 0;
			p.pose(2) = 0;
			p.vel(0) = 0;
			p.vel(1) = 0;
			p.vel(2) = 0;
		}
		return;
	}
	// use 300 first IMU measurements to
	// initialize IMU bias
	/*if (!inertial_init_) {
		if (inertial_queue_.size() == 300) {
			auto lambda_x = [&](Eigen::Vector3d a, sensor_msgs::ImuConstPtr b){
				a(0) += b->linear_acceleration.x;
		       		a(1) += b->linear_acceleration.y;
				return a;	};	
			imu_biases_ = std::accumulate(inertial_queue_.begin(), inertial_queue_.end(), Eigen::Vector3d{0,0,0}, lambda_x);
			imu_biases_ /= 300;
			std::cout << "IMU initialized: "<< imu_biases_.transpose() << std::endl;
			inertial_init_ = true;
		} else {
			inertial_queue_.push_back(state);
			return;
		}
	}
	*/
	Eigen::Vector3d vel_measurement;
	Eigen::Vector3d accel_measurement;
	// TODO: replace with:
	// tf::vectorMsgToEigen

	//vel_measurement[0] = state->fl;
	//vel_measurement[1] = state->rl;
	//vel_measurement[2] = state->rr;
	//vel_measurement[3] = state->fr;

	double delta_t = state->header.stamp.toSec() - last_state->header.stamp.toSec();

	// assuming 100 hz
	if(delta_t > 0.08) {
		ROS_WARN_STREAM_THROTTLE(3.0, "Prediction: dt " << delta_t << "s very high!");
	}

	static std::random_device rd{};
	static std::mt19937 rand_engine{rd()};

//	std::normal_distribution<double> rr_distribution(vel_measurement[2], noises_(0, 0)); //revs RR wheel
//	std::normal_distribution<double> rl_distribution(vel_measurement[1], noises_(1, 1)); //revs RL wheel
    std::normal_distribution<double> x_noise(0.0, noises_(0, 0));
	std::normal_distribution<double> y_noise(0.0, noises_(0, 1));
	std::normal_distribution<double> yaw_noise(0.0, noises_(1, 0));


	for (auto &p : particles_)
	{	
		if (&p == &particles_.back() && setting_gps_) {
			continue;
		}
		PROFC_NODE("predictparticle");
		p.history.push_back(p.pose);
		//accel_measurement[0] = ax_distribution(rand_engine);
		//accel_measurement[1] = ay_distribution(rand_engine);
		//accel_measurement[2] = az_distribution(rand_engine);

		double tire_radius = 0.224;
		double lidar_offset = 0.5;
		double trac = 1.225;
		// double r2 = wz_distribution(rand_engine);
		// double slip_angle = some_distribution(rand_engine);
		//double speed_fl = vel_measurement[0] * tire_radius;
		//double speed_rl = vel_measurement[1] * tire_radius;
		//double speed_rr = vel_measurement[2] * tire_radius;
		//double speed_fr = vel_measurement[3] * tire_radius;
        double speed_x = state->twist.twist.linear.x;
        double speed_y = state->twist.twist.linear.y;
        double speed_yaw = state->twist.twist.angular.z;

		p.vel(0) = speed_x;

		p.vel(1) = speed_y;


		p.pose(0) += (p.vel(0) * delta_t + x_noise(rand_engine)) * cos(p.pose(2)) - (p.vel(1) * delta_t + y_noise(rand_engine)) * sin(p.pose(2));

		p.pose(1) += (p.vel(0) * delta_t + x_noise(rand_engine)) * sin(p.pose(2)) + (p.vel(1) * delta_t + y_noise(rand_engine)) * cos(p.pose(2));

		p.pose(2) = tools_.normalize_angle(p.pose(2) + yaw_noise(rand_engine) + speed_yaw * delta_t);


	}

}

/**
 * Given a new cone, check all cones in particle and return with highest probability
 * p - particle
 * gx, gy - global coordinates of new observation (cone.xy + pose.xy) we want to match against
 * best_sig - not used, same as best_Q?
 * best_Q - covariance of best landmark
 * best_K - stan: i think this supposed to be kalman gain, but not used
 * best_H - jacobian of best landmark wrt motion model
 * best_z_diff - delta of range and bearing of best landmark
 * z - bearing and range of cone, const
 * nearest_distance -  not used, unknown
 * best_prob - probability of best landmark
 * l - id of best landmark
 * type - not used, probably meant as color of best landmark
 *
*/
void FastSlam::Associate_observation(
		const Particle &p, const double &gx, const double &gy,
		Eigen::Matrix2d &best_sig, Eigen::Matrix2d &best_Q, Eigen::Matrix2d &best_K,
		Eigen::Matrix2d &best_H, Eigen::Vector2d &best_z_diff,
		const RadarReading &z, double &nearest_distance, double &best_prob,
		int &l, int type)
{
	// for each landmark in the map of the particle
	for (auto &landmark : p.landmarks)
	{
		// get distance to it for global check
		const double dx = gx - landmark.mu(0);
		const double dy = gy - landmark.mu(1);
		// only consider certain distance
		if (std::hypot(dx, dy) < max_matching_distance_)
		{
			Eigen::Matrix2d H;
			Eigen::Vector2d z_actual, expect_Z;
			z_actual << z.range, z.bearing;
			Measurement_model(p, landmark.id, expect_Z, H);
			Eigen::Matrix2d Q;
		        Q.noalias() = H * landmark.sigma * H.transpose() + Q_;
			// calculate the error between the z and expected Z
			Eigen::Vector2d z_diff = z_actual - expect_Z;
			z_diff(1) = tools_.normalize_angle(z_diff(1));
			// calculate the matching probability
			// FIXME(Stan): check formula
			double prob =
					exp(-0.5 * z_diff.transpose() * Q.inverse() * z_diff) /
					((2 * M_PI) * sqrt(Q.determinant()));
			// ROS_WARN_STREAM("Propability " << prob << "!");
			if (prob > best_prob)
			{
				best_prob = prob;
				l = landmark.id;
				best_Q = Q;
				best_H = H;
				best_z_diff = z_diff;
			}
		}
	}
}

/**
 * p - particle
 * best_q - covariance of best landmark
 * best_H - jacobian of best landmark
 * best_z_diff - 
 * l
 * best_prob
 * type
*/
void FastSlam::update_landmark(Particle &p, const Eigen::Matrix2d &best_Q, const Eigen::Matrix2d &best_H, const Eigen::Vector2d &best_z_diff, const int &l, const double &best_prob, int type)
{

	// Kalman gain
	Eigen::Matrix2d K = p.landmarks[l].sigma * best_H.transpose() * best_Q.inverse();

	if (lap_ < 1 && !mapped_)
	{
		//update position and covariance of landmark (only in 1st lap)
		p.landmarks[l].mu = p.landmarks[l].mu + K * best_z_diff;
		p.landmarks[l].sigma = p.landmarks[l].sigma - K * best_H * p.landmarks[l].sigma;
		if (!sim_)
		{
			int norm = p.landmarks[l].alpha_l + p.landmarks[l].alpha_r + p.landmarks[l].alpha_u;
			//update landmark's color (only in 1st lap)
			if (type == 0)
			{
				p.landmarks[l].alpha_l++;
			}
			else if (type == 1)
			{
				p.landmarks[l].alpha_r++;
			}
			else if (type == 3)
			{
				p.landmarks[l].alpha_u++;
			}
			p.landmarks[l].color_probabilities[0] = (double)p.landmarks[l].alpha_l / (double)(norm + 1);
			p.landmarks[l].color_probabilities[1] = (double)p.landmarks[l].alpha_r / (double)(norm + 1);
			p.landmarks[l].color_probabilities[2] = (double)p.landmarks[l].alpha_u / (double)(norm + 1);
		}
	}
	if (!sim_)
	{
		auto posterior_predictive = std::max_element(p.landmarks[l].color_probabilities.begin(), p.landmarks[l].color_probabilities.end());
		p.landmarks[l].type = std::distance(p.landmarks[l].color_probabilities.begin(), posterior_predictive);
	}
	// reweight the particle
	p.weight *= best_prob;
}

void FastSlam::add_landmark(Particle &p, const double &gx, const double &gy, RadarReading &z, int type)
{
	int l = p.landmarks.size();
	// intialize landmark
	LandMark newLandmark;
	z.id = newLandmark.id = l;
	p.landmarks.push_back(newLandmark);
	p.landmarks[l].mu << gx, gy;
	// get the Jacobian with respect to the landmark position
	Eigen::Matrix2d H;
	Eigen::Vector2d h;
	Measurement_model(p, z.id, h, H);
	// initialize the ekf for this landmark
	p.landmarks[l].sigma = H.inverse() * Q_ * (H.inverse()).transpose();
	p.landmarks[l].observed = true;
	if (!sim_)
	{
		p.landmarks[l].type = type;
	}
	// calculate the weight
	p.weight *= p_new_landmark_;
}
//correction for live execution
void FastSlam::Correction(const custom_msgs::ConeArrayConstPtr &temp_cloud)
{
	PROFC_NODE("correctiontotal");

    if (last_pred_time_) {
        double correction_delay = last_pred_time_ - temp_cloud->header.stamp.toSec();
        //ROS_WARN_STREAM("Correction has delay tp Prediction " << correction_delay);
    }

	// check delay
	// 15 is for because we have lidar scans every 10ms
	// but we also add some  +/- 5ms slack
	double diff_sec = std::abs(temp_cloud->header.stamp.toSec() - last_cones_time.toSec());
	if ( diff_sec > 0.15 && is_initialized_) {
		ROS_WARN_STREAM("Correction: Skipped lidar frame! time between frame > 0.15 " << diff_sec);	
	} 
	last_cones_time = temp_cloud->header.stamp;

	// if not using OpenMP, use:
	//for (auto &p : particles_)
	omp_set_dynamic(0); 
	omp_set_num_threads(4); // equal amount of particles_ per thread
#pragma omp parallel for
	for (auto p = particles_.begin(); p < particles_.end(); p++)
	{
	    if(!mapped_)
	        loadMapFromFile(*p);

		if(!is_initialized_ && loc_mode_)
		{
			//initialize_location(*p,temp_cloud);			
		}				
		const Eigen::Vector3d &robot = p->pose;
		int observation_size = temp_cloud->cones.size();

		// for each cone:
		for (int i = 0; i < observation_size; ++i)
		{
			int l;
			// transform to Range and bearing
			RadarReading z;
			// Simon: I guess here we rotate the lidar cloud...
			// Jan: No
			auto x = temp_cloud->cones[i].position.x + x_offset_;
			auto y = temp_cloud->cones[i].position.y + y_offset_;
			z.range = std::sqrt(x*x + y*y);
			// only considers observations that are realistic
			if (z.range > max_considered_distance_ || z.range < min_considered_distance_) // TODO: x<0 ignores cones behind the lidar
			{
				continue;
			}
			z.bearing = tools_.normalize_angle(std::atan2(y,x));

			//motion undistorsion
			//double dt = (tools_.normalize_angle(M_PI + z.bearing) * (0.05 / M_PI));

			//x += p->vel(0) * ((z.bearing) * (0.05 / M_PI));
			//y += p->vel(1) * ((z.bearing) * (0.05 / M_PI));
			//z.range = std::sqrt(x*x + y*y);
			//z.bearing += p->vel(2) * ((z.bearing) * (0.05 / M_PI));



			// TODO
			// check if cone is "behind" the lidar
			// or in some other bad location
			/*
			if (z.bearing > M_PI / 2 or z.bearing < -M_PI / 2) {
				ROS_WARN_STREAM("Bearing: " << z.bearing << "! Cone "<< x << " " << y <<" is behind front wing");
				continue;
			} */

			// observation transformed to global coordinates
			double gx = robot(0) + z.range * cos(robot(2) + z.bearing);
			double gy = robot(1) + z.range * sin(robot(2) + z.bearing);
			double best_prob = 0.0;
			Eigen::Matrix2d best_sig, best_Q, best_K, best_H;
			Eigen::Vector2d best_z_diff;
			double nearest_distance;

			Associate_observation(*p, gx, gy, best_sig, best_Q, best_K, best_H, best_z_diff, z, nearest_distance, best_prob,
					l, 3);
			// if we found a cone with very high match probability, update landmark
			if (best_prob > p_new_landmark_)
			{
				update_landmark(*p, best_Q, best_H, best_z_diff, l, best_prob, 3);
			}
			//else new landmark
			else
			{
				if (lap_ < 1 && !mapped_)
				{
					add_landmark(*p, gx, gy, z, 3);
				}
				else
				{
					p->weight *= 0.2;
				}
			}
		} //for each observation
	} //for each particle
	is_initialized_ = true;
}

void FastSlam::Resample()
{
	PROFC_NODE("resample");
	static std::random_device rd{};
	static std::mt19937 gen{rd()};

	for (int i = 0; i < N_; i++)
	{
		eigen_weights_(i) = particles_[i].weight;
	}
	double sum = eigen_weights_.sum();
	eigen_weights_ = eigen_weights_ / sum;

	std::discrete_distribution<> d(eigen_weights_.data(), eigen_weights_.data() + eigen_weights_.size());
	vector<Particle, Eigen::aligned_allocator<Particle>> resampParticle;

	resampParticle.reserve(N_);
	int idx;
	idx = MaxIndex();
	double nwei = 1.0 / N_;
	for (unsigned int i = 0; i < N_; i++)
	{
		//idx = d(gen);
		resampParticle.push_back(particles_[idx]);
		resampParticle[i].weight = nwei;
	}
	particles_ = std::move(resampParticle);
}

void FastSlam::getBestPoseAndLandmark(VectorXd &mu_)
{
	int index = MaxIndex();
	Particle &p = particles_[index];
	int M = p.landmarks.size();
	mu_ = Eigen::VectorXd::Zero(2 * M + 3 + 2);
	mu_(0) = p.pose(0);
	mu_(1) = p.pose(1);
	mu_(2) = p.pose(2);
	mu_(3) = p.vel(0);
	mu_(4) = p.vel(1);
	for (int i = 0; i < M; i++)
	{
		mu_(2 * i + 5) = p.landmarks[i].mu(0);
		mu_(2 * i + 6) = p.landmarks[i].mu(1);
	}
}

//intialize_location for sim
/*void FastSlam::intialize_location(Particle &p, const pcl::PointCloud<pcl::PointXYZ>::Ptr &temp_cloud)
{
	Eigen::Vector2d n_left, n_right;
	Eigen::Vector2d p_end_left, p_end_right;
	int obs_left, obs_right;
	int lm_left, lm_right;
	double range_obs_left = 10.0, range_obs_right = 10.0, range_lm_left = 10.0, range_lm_right = 10.0;

	for (int i = 0; i < temp_cloud->points.size(); ++i)
	{
		//getting observation
		double temp_range = std::hypot(temp_cloud->points[i].x, temp_cloud->points[i].y);
		if (temp_range < range_obs_left && temp_cloud->points[i].y > 0)
		{ //left cone
			range_obs_left = temp_range;
			obs_left = i;
		}
		if (temp_range < range_obs_right && temp_cloud->points[i].y < 0)
		{ //right cone
			range_obs_right = temp_range;
			obs_right = i;
		}
		//getting landmarks
		temp_range = std::hypot(p.landmarks[i].mu(0), p.landmarks[i].mu(1));
		if (temp_range < range_lm_left && p.landmarks[i].mu(1) > 0)
		{ //left cone
			range_lm_left = temp_range;
			lm_left = i;
		}
		if (temp_range < range_lm_right && p.landmarks[i].mu(1) < 0)
		{ //right cone
			range_lm_right = temp_range;
			lm_right = i;
		}
	}

	n_left << temp_cloud->points[obs_left].x / range_obs_left, temp_cloud->points[obs_left].y / range_obs_left;
	n_right << temp_cloud->points[obs_right].x / range_obs_right, temp_cloud->points[obs_right].y / range_obs_right;

	p_end_left << p.landmarks[lm_left].mu(0) + n_left(0), p.landmarks[lm_left].mu(1) + n_left(1);
	p_end_right << p.landmarks[lm_right].mu(0) + n_right(0), p.landmarks[lm_right].mu(1) + n_right(1);

	Eigen::Hyperplane<float, 2> left = Eigen::Hyperplane<float, 2>::Through(p.landmarks[lm_left].mu, p_end_left), 
	Eigen::Hyperplane<float, 2> right = Eigen::Hyperplane<float, 2>::Through(p.landmarks[lm_right].mu, p_end_right);
	std::cout << "Intersection:\n"
						<< left.intersection(right) << '\n';
}*/

//initialize_location for live execution
void FastSlam::initialize_location(Particle &p, const custom_msgs::ConeArrayConstPtr &temp_cloud) const
{
	int obs_left, obs_right;
	int lm_left, lm_right;
	double range_obs_left = 10.0, range_obs_right = 10.0, range_lm_left = 10.0, range_lm_right = 10.0;

	std::cout << "init" << std::endl;
	std::cout << "size: " << temp_cloud->cones.size() << std::endl;
	for (int i = 0; i < temp_cloud->cones.size(); ++i)
	{
		std::cout << "cone: " << i << std::endl;
		//getting observation
		double temp_range = std::hypot(temp_cloud->cones[i].position.x, temp_cloud->cones[i].position.y);
		if (temp_range < range_obs_left && temp_cloud->cones[i].position.y > 0)
		{ //left cone
			range_obs_left = temp_range;
			obs_left = i;
		}
		if (temp_range < range_obs_right && temp_cloud->cones[i].position.y < 0)
		{ //right cone
			range_obs_right = temp_range;
			obs_right = i;
		}
		//getting landmarks
		temp_range = std::hypot(p.landmarks[i].mu(0), p.landmarks[i].mu(1));
		if (temp_range < range_lm_left && p.landmarks[i].mu(1) > 0)
		{ //left cone
			range_lm_left = temp_range;
			lm_left = i;
		}
		if (temp_range < range_lm_right && p.landmarks[i].mu(1) < 0)
		{ //right cone
			range_lm_right = temp_range;
			lm_right = i;
		}
	}

	Eigen::Vector2d n_left((temp_cloud->cones[obs_left].position.x / range_obs_left), (temp_cloud->cones[obs_left].position.y / range_obs_left));
	Eigen::Vector2d n_right((temp_cloud->cones[obs_right].position.x / range_obs_right), (temp_cloud->cones[obs_right].position.y / range_obs_right));
	

	Eigen::Vector2d p_end_left((p.landmarks[lm_left].mu(0) + n_left(0)), (p.landmarks[lm_left].mu(1) + n_left(1)));
	Eigen::Vector2d p_end_right((p.landmarks[lm_right].mu(0) + n_right(0)), (p.landmarks[lm_right].mu(1) + n_right(1)));
	

	Eigen::Hyperplane<double, 2> left = Eigen::Hyperplane<double, 2>::Through(p.landmarks[lm_left].mu, p_end_left);
	Eigen::Hyperplane<double, 2> right = Eigen::Hyperplane<double, 2>::Through(p.landmarks[lm_right].mu, p_end_right);
	std::cout << "Intersection:\n"<< left.intersection(right) << '\n';
}

int FastSlam::MaxIndex()
{
	int max_Index = 0;
	double max_val = 0.0;
	for (size_t i = 1; i < particles_.size(); i++)
	{
		if (max_val < particles_[i].weight)
		{
			max_val = particles_[i].weight;
			max_Index = i;
		}
	}
	return max_Index;
}

/**
* returns whether the RMS covariance (xx and yy) for a landmark are below the configured threshold
*/
bool FastSlam::isConeBelowThreshold(const LandMark &landmark)
{
    return (sqrt(
                pow(landmark.sigma(0, 0), 2) +
                pow(landmark.sigma(1, 1), 2)
            ) < cone_covariance_threshold_);
}
