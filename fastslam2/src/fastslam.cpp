#include "fastslam.h"
#include "profc.h"
#include <omp.h>
/*
 * Constructor.
 */
FastSlam::FastSlam() { is_initialized_ = false; }

const double rear_axle_offset = 0.6; //on master branch: 1.65
std::random_device FastSlam::rd_;
std::mt19937 FastSlam::rand_engine_(FastSlam::rd_());

/**
Initialize the parameters
**/
void FastSlam::Initialize(ros::NodeHandle nh)
{
	std::map<std::string, double> motion_noise_map3D, Q_map, offset_map;
	nh.getParam("motionNoise3D", motion_noise_map3D);
	nh.getParam("Q", Q_map);
	nh.getParam("offset", offset_map);
	nh.getParam("amz_sim", amz_sim_);
	nh.getParam("loc_mode", loc_mode_);
	nh.getParam("gps_fusion", setting_gps_);
	nh.getParam("cone_covariance_threshold", cone_covariance_threshold_);
	nh.getParam("min_cone_covariance", min_cone_covariance_);
	nh.getParam("cones_minimum_dist", cones_minimum_dist_);
	nh.getParam("cone_counter", cone_counter_);
	nh.getParam("punish_weight", punish_weight_);
	nh.getParam("wait_time_after_closure", wait_time_after_closure_);
	nh.getParam("map_cleaning", map_cleaning_);


	if (setting_gps_) 
	{
		ROS_WARN_STREAM("Using GPS fusion!");
	} else {
		ROS_WARN_STREAM("GPS fusion Disabled!");
	}

    motion_noise_3D_ = Eigen::Matrix3d::Zero();
    motion_noise_3D_(0, 0) = motion_noise_map3D["r00"];
    motion_noise_3D_(0, 1) = motion_noise_map3D["n01"];
    motion_noise_3D_(0, 2) = motion_noise_map3D["n02"];
    motion_noise_3D_(1, 0) = motion_noise_map3D["n10"];
    motion_noise_3D_(1, 1) = motion_noise_map3D["r11"];
    motion_noise_3D_(1, 2) = motion_noise_map3D["n12"];
    motion_noise_3D_(2, 0) = motion_noise_map3D["n20"];
    motion_noise_3D_(2, 1) = motion_noise_map3D["n21"];
    motion_noise_3D_(2, 2) = motion_noise_map3D["r22"];

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
	if (!loc_mode_) // Erik: what does relocalizing mean here?
	{
		// not relocalizing...
		particles_.resize(N_);
		for (int i = 0; i < N_; i++)
		{
			particles_[i].weight = 1.0 / N_;
			particles_[i].pose << 0, 0, 0;  //this  and
			particles_[i].vel << 0, 0, 0;   //this too
			particles_[i].landmarks = vector<LandMark,Eigen::aligned_allocator<LandMark>>();
			particles_[i].id=i;
			particles_[i].mu << 0, 0, 0;
			particles_[i].sampledPose << 0, 0, 0;
			//particles_[i].sigma = motion_noise_3D_;
			particles_[i].sigma = Eigen::Matrix3d::Zero();
			particles_[i].data_associations = vector<DataAssociation, Eigen::aligned_allocator<DataAssociation>>();

		}
	}
	else
	{
		particles_.resize(N_);
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

				if (!amz_sim_)
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

// this function is only needed if IMU data is used for prediction. This function filters out the IMU bias
void FastSlam::InitializeIMU(sensor_msgs::ImuConstPtr state){

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

/**
 * Prediction step using wheel revolutions
 */
void FastSlam::Prediction2(const nav_msgs::Odometry* state,
                          const nav_msgs::Odometry* last_state,
                          boost::shared_ptr<const nav_msgs::Odometry> ins_vel)
{
	PROFC_NODE("prediction");
    last_pred_time_ = state->header.stamp.toSec();

	if ( ins_vel == nullptr ) {
    	//std::cout << "No GPS" << std::endl;
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

	double speed_x = state->twist.twist.linear.x;
    double speed_y = state->twist.twist.linear.y;
    double speed_yaw = state->twist.twist.angular.z;


	double delta_t = state->header.stamp.toSec() - last_state->header.stamp.toSec();

	// assuming 100 hz
	if(delta_t > 0.05) {
		//ROS_WARN_STREAM("Prediction: dt " << delta_t << "s very high!");
	}

	for (auto &p : particles_){

		if (&p == &particles_.back() && setting_gps_) {
			continue;
		}
		PROFC_NODE("predictparticle");
		p.history.push_back(p.pose);

		p.vel(0) = speed_x;

		p.vel(1) = speed_y;

		p.vel(2) = speed_yaw;

		p.pose(0) += p.vel(0) * delta_t * cos(p.pose(2)) - p.vel(1) * delta_t * sin(p.pose(2));

		p.pose(1) += p.vel(0) * delta_t * sin(p.pose(2)) + p.vel(1) * delta_t * cos(p.pose(2));

		p.pose(2) = tools_.normalize_angle(p.pose(2) + p.vel(2) * delta_t);



		Eigen::Matrix3d JacobianWrtPose, JacobianWrtControl;
		JacobianWrtControl.setZero();
        JacobianWrtPose.setZero();

		JacobianWrtPose << 1, 0, -sin(p.pose(2)) * (p.vel(0) * delta_t) - (p.vel(1) * delta_t) * cos(p.pose(2)),
		                   0, 1,  cos(p.pose(2)) * (p.vel(0) * delta_t) - (p.vel(1) * delta_t) * sin(p.pose(2)),
		                   0, 0, 1;

        JacobianWrtControl <<   cos(p.pose(2))*delta_t, -sin(p.pose(2))*delta_t, 0,
		                        sin(p.pose(2))*delta_t, cos(p.pose(2))*delta_t,  0,
		                        0, 0, delta_t;
        /*
	    p.sigma = JacobianWrtPose * p.sigma * JacobianWrtPose.transpose() + JacobianWrtControl * motion_noise_3D_ * JacobianWrtControl.transpose();

		*/

        // initiate distribution
        // because we have several prediction calls before a correction we need to add up the uncertainty
        p.mu = p.pose;
		p.sigma = JacobianWrtPose * p.sigma * JacobianWrtPose.transpose() + motion_noise_3D_;
	}

}

/**
 * measurement_model
 * for current pose and landmark
 * p - particle
 * id - which landmark
 * h - measurement - output
 * H - Jacobian - output
 */
 //Erik: Calculates the expected position of the landmark with the id "id" relative to the particle "p"
void FastSlam::Measurement_model(const Particle &p, const int &id, Eigen::Vector2d &h, Eigen::Matrix2d &featureJacobian, Eigen::Matrix<double,2,3> &poseJacobian)
{
	auto pose = p.landmarks[id].mu; //mean position of landmark
	double delta_x = pose(0) - p.pose(0); //x-distance of landmark to particle
	double delta_y = pose(1) - p.pose(1);   //y-distance of landmark to particle
	// bearing and range to selected cone
	double expect_range = sqrt(delta_x * delta_x + delta_y * delta_y); //range from landmark to particle
	double expect_bearing =
			tools_.normalize_angle(atan2(delta_y, delta_x) - p.pose(2)); // angle of landmarks position in particles frame
	h << expect_range, expect_bearing;

	//jacobian of measurement function wrt to feature
    featureJacobian = Eigen::Matrix2d::Zero();
	featureJacobian(0, 0) = delta_x / expect_range;
	featureJacobian(0, 1) = delta_y / expect_range;
	featureJacobian(1, 0) = -delta_y / (expect_range * expect_range);
	featureJacobian(1, 1) = delta_x / (expect_range * expect_range);
    //jacobian wrt to pose
	poseJacobian = Eigen::Matrix<double,2,3>::Zero();
	poseJacobian(0,0) = -delta_x / expect_range;
	poseJacobian(0,1) = -delta_y / expect_range;
	poseJacobian(0,2) = 0.0;
	poseJacobian(1,0) = delta_y / (expect_range * expect_range);
	poseJacobian(1,1) = -delta_x / (expect_range * expect_range);
	poseJacobian(1,2) = -1.0;
}


void FastSlam::AdjustProposalDist(Particle &p, const RadarReading &z){

    // iteratively adjust the distribution with every observation
    Eigen::Vector2d z_actual;
    z_actual<< z.range, z.bearing;
    double gx = p.pose(0) + z.range * cos(p.pose(2) + z.bearing);
    double gy = p.pose(1) + z.range * sin(p.pose(2) + z.bearing);
    for (auto &landmark : p.landmarks)
    {
        //get the distance of each landmark in the particles map to the given observation (gx,gy)
        const double dx = gx - landmark.mu(0);
        const double dy = gy - landmark.mu(1);
        //only consider certain distance. If the landmark is further away than max_matching_distance_ its most likely not the same landmark as (gx,gy)
        if (std::hypot(dx, dy) <= max_matching_distance_)
        {
            Eigen::Matrix<double,2,3> myPoseJacobian;
            Eigen::Matrix2d myFeatureJacobian, Z;
            Eigen::Vector2d expect_z;
            // Erik: Calculates the expected position of the landmark with the id "landmark.id" relative to the particle "p" (local frame)
            Measurement_model(p, landmark.id, expect_z, myFeatureJacobian, myPoseJacobian);
            // calculate the error between the z and expected Z
            Eigen::Vector2d z_diff = z_actual - expect_z;
            z_diff(1) = tools_.normalize_angle(z_diff(1));
            // Covariance Matrix associated with landmarks position, Q_ is measurement noise
            Z.noalias() = myFeatureJacobian * landmark.sigma * myFeatureJacobian.transpose() + Q_;
            p.sigma = (myPoseJacobian.transpose() * Z.inverse() * myPoseJacobian + p.sigma.inverse()).inverse();
            p.mu = p.mu + p.sigma * myPoseJacobian.transpose() * Z.inverse() * z_diff;
        }
    }
}

//TODO(ERIK) Implement Mutual Exclusion with JCBB
void FastSlam::Associate_observation(Particle &p, DataAssociation &da)
{
    RadarReading z = da.reading;
    double gx = p.pose(0) + z.range * cos(p.pose(2) + z.bearing);
	double gy = p.pose(1) + z.range * sin(p.pose(2) + z.bearing);
	// for each landmark in the map of the particle
	for (auto &landmark : p.landmarks)
	{
		// get distance to it for global check
		const double dx = gx - landmark.mu(0);
		const double dy = gy - landmark.mu(1);
		// only consider certain distance
		if (std::hypot(dx, dy) <= max_matching_distance_)
		{
			Eigen::Matrix<double,2,3> myPoseJacobian;
			Eigen::Matrix2d myFeatureJacobian, Z;
			Eigen::Vector2d z_actual, expect_z;
			z_actual << z.range, z.bearing;
			Measurement_model(p, landmark.id, expect_z, myFeatureJacobian, myPoseJacobian);
		    Z.noalias() = myFeatureJacobian * landmark.sigma * myFeatureJacobian.transpose() + Q_;
            Eigen::Vector2d z_diff = z_actual - expect_z;
            z_diff(1) = tools_.normalize_angle(z_diff(1));


			// calculate the error between the z and expected z with the sampled pose
            double delta_x = landmark.mu(0) - p.sampledPose(0); //x-distance of landmark to particle
            double delta_y = landmark.mu(1) - p.sampledPose(1);   //y-distance of landmark to particle
            // bearing and range to selected cone
            double expect_range = sqrt(delta_x * delta_x + delta_y * delta_y); //range from landmark to particle
            double expect_bearing = tools_.normalize_angle(atan2(delta_y, delta_x) - p.sampledPose(2)); // angle of landmarks position in particles frame
            expect_z << expect_range, expect_bearing;
            Eigen::Vector2d z_diff_new = z_actual - expect_z;
            z_diff_new(1) = tools_.normalize_angle(z_diff_new(1));


			// calculate the matching probability
			double prob =exp(-0.5 * z_diff_new.transpose() * Z.inverse() * z_diff_new) /sqrt((2 * M_PI* Z).determinant());
			if (prob > da.probability)
			{
				da.landmark = landmark;
				da.probability = prob;
				da.InnovationMatrix = Z;
				da.FeatureJacobian = myFeatureJacobian;
				da.PoseJacobian = myPoseJacobian;
				da.Measurement_diff = z_diff;
			}
		}
	}
}


//update position and covariance of landmark (only in 1st lap)
void FastSlam::update_landmark(Particle &p, int type, const DataAssociation &da)
{
	Eigen::Matrix2d featureJacobian, Z, L;

	Z = da.InnovationMatrix;
	featureJacobian = da.FeatureJacobian;
	Eigen::Matrix<double,2,3> poseJacobian = da.PoseJacobian;
	Eigen::Vector2d z_diff = da.Measurement_diff;

    int l_id= da.landmark.id;

    if (lap_ < 1 && !found_loop_closure_ && !mapped_)
    {
        //update position and covariance of landmark (only in 1st lap)
        Eigen::Matrix2d K;
        K.noalias() = p.landmarks[l_id].sigma * featureJacobian.transpose() * Z.inverse();
        p.landmarks[l_id].mu = p.landmarks[l_id].mu + K * z_diff;
        p.landmarks[l_id].sigma = p.landmarks[l_id].sigma - K * featureJacobian * p.landmarks[l_id].sigma;
        if(p.landmarks[l_id].sigma(0,0) < min_cone_covariance_)
            p.landmarks[l_id].sigma(0,0) = min_cone_covariance_;
        if(p.landmarks[l_id].sigma(1,1) < min_cone_covariance_)
            p.landmarks[l_id].sigma(1,1) = min_cone_covariance_;
        p.landmarks[l_id].counter += cone_counter_; //Erik: add two for stability reasons. If only 1 is added the map changes to frequently for the planner to do global planning
        if (!amz_sim_) //Erik: amz_sim set to false in yaml file
        {
            int norm = p.landmarks[l_id].alpha_l + p.landmarks[l_id].alpha_r + p.landmarks[l_id].alpha_u;
            //update landmark's color (only in 1st lap)
            if (type == 0)
            {
                p.landmarks[l_id].alpha_l++;
            }
            else if (type == 1)
            {
                p.landmarks[l_id].alpha_r++;
            }
            else if (type == 3)
            {
                p.landmarks[l_id].alpha_u++;
            }
            p.landmarks[l_id].color_probabilities[0] = (double)p.landmarks[l_id].alpha_l / (double)(norm + 1);
            p.landmarks[l_id].color_probabilities[1] = (double)p.landmarks[l_id].alpha_r / (double)(norm + 1);
            p.landmarks[l_id].color_probabilities[2] = (double)p.landmarks[l_id].alpha_u / (double)(norm + 1);
        }
    }
    if (!amz_sim_) //Erik: amz_sim set to false in yaml file
    {
        auto posterior_predictive = std::max_element(p.landmarks[l_id].color_probabilities.begin(), p.landmarks[l_id].color_probabilities.end());
        p.landmarks[l_id].type = std::distance(p.landmarks[l_id].color_probabilities.begin(), posterior_predictive);
    }
    // reweigh the particle in proportion to how big z_diff was. If z_diff is small the weight should be bigger
    L = poseJacobian * motion_noise_3D_ * poseJacobian.transpose() + Z;
    p.weight *= exp(-0.5 * z_diff.transpose() * L.inverse() * z_diff) /sqrt((2 * M_PI* L).determinant());
}

void FastSlam::add_landmark(Particle &p, int type, const DataAssociation &da)
{
    //std::cout << "entered add_landmark" << std::endl;
    float r = da.reading.range;
    double c = cos(p.pose(2) + da.reading.bearing);
    double s = sin(p.pose(2) + da.reading.bearing);
    double gx = p.pose(0) + r * c;
    double gy = p.pose(1) + r * s;

    /*
    for(auto &lm : p.landmarks){
        float dist = sqrt(pow(gx - lm.mu(0), 2) + pow(gy - lm.mu(1), 2));
        if(dist < cones_minimum_dist_){
            p.weight *= punish_weight_ * punish_weight_;
            return;
        }
    }
    */

    int l = p.landmarks.size();
    // initialize landmark
    LandMark newLandmark;
    newLandmark.id = l;
    p.landmarks.push_back(newLandmark);
    p.landmarks[l].mu << gx, gy;
    // get the Jacobian with respect to the landmark position
    Eigen::Matrix2d inverseFeatureJacobian = Eigen::Matrix2d::Zero();

	inverseFeatureJacobian << c, -r * s,
	                          s,  r * c;

    // initialize the ekf for this landmark
    p.landmarks[l].sigma.noalias() = inverseFeatureJacobian * Q_ * inverseFeatureJacobian.transpose();
    p.landmarks[l].observed = true;
    p.landmarks[l].counter = 1;
    if (!amz_sim_)
    {
        p.landmarks[l].type = type;
    }
    //reduce weight of particle for every newly added cone, to increase prob. of associating existing cones instead of adding new ones
    p.weight *= punish_weight_;
}

void FastSlam::decrease_landmark_counter(Particle &p){
    for (auto &landmark : p.landmarks) // for each landmark in the map of the particle
	{
	    auto pose = landmark.mu; //mean position of landmark
	    double delta_x = pose(0) - p.pose(0); //x-distance of landmark to particle
	    double delta_y = pose(1) - p.pose(1);   //y-distance of landmark to particle
	    // bearing and range to selected cone
	    double expected_range = sqrt(delta_x * delta_x + delta_y * delta_y); //range from landmark to particle
	    double expected_bearing = tools_.normalize_angle(atan2(delta_y, delta_x) - p.pose(2)); // angle of landmarks position in particles fram
		//if landmark is visible decrease its counter
	    if(expected_bearing < (M_PI / 2) && expected_bearing > (-M_PI / 2) && expected_range < max_considered_distance_ && expected_range > min_considered_distance_){
	        //expected_bearing < (M_PI / 3) && expected_bearing > (-M_PI / 3) && expected_range < 0.5*max_considered_distance_ && expected_range > min_considered_distance_
            landmark.counter -= 1;
	    }
	}

}

void FastSlam::remove_landmark(Particle &p){

    bool flag = false;
    int c = 0;
	p.landmarks.erase(
    std::remove_if(p.landmarks.begin(), p.landmarks.end(),
        [p, &flag, &c](const LandMark &l) { if(l.counter<0){
                                                flag = true;
                                                c++;}
                                                return (l.counter<0); }),
    p.landmarks.end());
    // after deletion of landmarks the order of ids might be wrong. This can lead to problems when adding new landmarks
    // example: landmark id before removal 123456789, after removal 12345679, after adding new landmark 123456799, now you have twice the same id...
    // thats why assign new ids. Also id must correspond to index in vector, else we could go out of bound.
    if(flag){
        int i = 0;
        for (auto &landmark : p.landmarks)
        {
            landmark.id = i++;
        }
        p.weight *= pow(punish_weight_,c);
    }
}

void FastSlam::CleanMap(Particle &p){

    remove_landmark(p);

    bool flag = false;
    for (int i =0; i<p.landmarks.size(); i++){
        for (int j =0; j<p.landmarks.size(); j++){
          if(i != j){
              float dist = sqrt(pow(p.landmarks[i].mu(0) - p.landmarks[j].mu(0), 2) + pow(p.landmarks[i].mu(1) - p.landmarks[j].mu(1), 2));
              if (dist < cones_minimum_dist_){
                p.weight *= punish_weight_;
                flag = true;
                p.landmarks.erase(p.landmarks.begin()+j);
              }
          }
        }
    }

    /*
    bool flag = false;
    float punish_weight = punish_weight_;
    float cones_minimum_dist = cones_minimum_dist_;
    p.landmarks.erase(
    std::remove_if(p.landmarks.begin(), p.landmarks.end(),
        [&p, &flag, punish_weight, cones_minimum_dist](const LandMark &l)
        { for (int j =0; j<p.landmarks.size(); j++){
            if(l.id != j){
                float dist = sqrt(pow(l.mu(0) - p.landmarks[j].mu(0), 2) + pow(l.mu(1) - p.landmarks[j].mu(1), 2));
                if(dist<cones_minimum_dist || l.counter<0){
                   p.weight *= punish_weight;
                   flag = true;
                   return true;}
                return false; }
          }
        }),p.landmarks.end());
    */
    // after deletion of landmarks the order of ids might be wrong. This can lead to problems when adding new landmarks
    // example: landmark id before removal 123456789, after removal 12345679, after adding new landmark 123456799, now you have twice the same id...
    // thats why assign new ids. Also id must correspond to index in vector, else we could go out of bound.
    if(flag){
        int i = 0;
        for (auto &landmark : p.landmarks)
        {
            landmark.id = i++;
        }
    }


}

//correction for live execution
//loops through all particles and all observations. Updates the Covariances of observed landmarks. Adds new landmarks to particles map.
//Reweights the particles, giving the ones with good maps higher weights. Good map = predicted position of landmarks matches observations well.

void FastSlam::Correction(const custom_msgs::ConeArrayConstPtr &temp_cloud)
{
	PROFC_NODE("correctiontotal");
	//std::cout << "entered Correction" << std::endl;


    if (last_pred_time_) {
        double correction_delay = last_pred_time_ - temp_cloud->header.stamp.toSec();
        //ROS_WARN_STREAM("Correction has delay to Prediction " << correction_delay);
    }

	// check delay
	// 15 because we have lidar scans every 10ms
	// but we also add some  +/- 5ms slack
	double diff_sec = std::abs(temp_cloud->header.stamp.toSec() - last_cones_time.toSec());
	if ( diff_sec > 0.15 && is_initialized_) {
		ROS_WARN_STREAM("Correction: Skipped lidar frame! time between frame > 0.15 " << diff_sec);	
	} 
	last_cones_time = temp_cloud->header.stamp;

	for (auto &p : particles_) //
	{

        // this seems to be wrong, we dont what to do that every iteration... mapped_ needs to be set to true directly after entering the function.
	    if(!mapped_)
	        loadMapFromFile(p);

		if(!is_initialized_ && loc_mode_) //Erik: loc_mode set to false in yaml file
		{
			//initialize_location(*p,temp_cloud);			
		}
		int observation_size = temp_cloud->cones.size();

        if (lap_ < 1 && !found_loop_closure_ && !mapped_){
            decrease_landmark_counter(p);
        }

        RadarReading obs[observation_size];

        // number of cones in field of view
        int M = 0;
        for (int i = 0; i < observation_size; i++)
		{
            auto x = temp_cloud->cones[i].position.x + x_offset_; //x_offset and y_offset are 0
            auto y = temp_cloud->cones[i].position.y + y_offset_;
            //range and bearing are relative to the particle, meaning in a local frame
            obs[i].range = sqrt(x*x + y*y);
            if (obs[i].range > max_considered_distance_ || obs[i].range < min_considered_distance_){
                obs[i].id = -1;
                continue;
            }
            obs[i].bearing = tools_.normalize_angle(std::atan2(y,x));
            obs[i].id = i;
            // iteratively adjusting the distribution and sample new pose from it
            AdjustProposalDist(p, obs[i]);
            M++;
        }

        // sample new pose from distribution and save the pose for later
        Eigen::MatrixXd sampled_pose;
        multivariate_normal(sampled_pose, p.mu, p.sigma);
        sampled_pose(2) = tools_.normalize_angle(sampled_pose(2));
        p.sampledPose = sampled_pose;
        Eigen::Vector3d prior_pose = p.pose;

        //important to set the covariance to zero again
        p.sigma = Eigen::Matrix3d::Zero();

        //p.pose = sampled_pose;

        //p.data_associations.reserve(M);
		// for each cone;
		for (int i = 0; i < observation_size; ++i)
		{
            if (obs[i].id == -1)
            {
               continue;
            }
            DataAssociation da;
            da.probability = 0.0;
            da.reading = obs[i];

            //checks the observation against all landmarks in the particles map
            //and returns the probability of the most likely data association and the corresponding landmark
            p.pose = prior_pose;
            Associate_observation(p, da); //for each landmark in particle
            p.pose = p.sampledPose;
            //p.data_associations.push_back(da);

            if (da.probability > p_new_landmark_){
                update_landmark(p, 3, da);
            }else{
                if (lap_ < 1 && !found_loop_closure_ && !mapped_)
                {
                    add_landmark(p, 3, da);
                }
                else
                {
                //reduce the weight of the particle, because it failed to associate an observation with a landmark,
                //although all landmarks must have been seen after the first lap
                    p.weight *= punish_weight_;
                }
            }
        }
        //clear all data associations to avoid copying them in the resampling step
        //p.data_associations.clear();
        //remove landmarks that are to close to each other or were not associated multiple times
        if (lap_ < 1 && !found_loop_closure_ && !mapped_){
            if(map_cleaning_)
                CleanMap(p);
        }

	}// for each particle

	is_initialized_ = true;
}

//keep the particles with the highest probabilities, so the ones whose maps coincide best with the observations, and overwrite the bad particles.
void FastSlam::Resample()
{
    PROFC_NODE("resample");
    //std::cout << "entered Resample" << std::endl;
    //normalize weights so that 0<weights<1 and sum(weights)=1
    double sum=0;
    for (int i = 0; i < N_; i++)
    {
        eigen_weights_(i) = particles_[i].weight;
        sum += eigen_weights_(i);
    }
    eigen_weights_ = eigen_weights_ / sum;
    //calculate the effective number of particles
    Eigen::Matrix<double, N_, 1> eigen_weights_squared = eigen_weights_.array().square();
    double Neff = 1/eigen_weights_squared.sum(); //effective number of particles
    //only resample, if the effective number of particles goes below 70% (arbitrary) of total number of particles
    if(Neff < Neff_limit_){
        //assign normalized weights to particles
        for (int i = 0; i < N_; i++){
            particles_[i].weight = eigen_weights_(i);
        }
        // debug_.safeParticleWeights(0);
        if(lvResample_){
            LVResample();
        }else{
            std::discrete_distribution<> d(eigen_weights_.data(), eigen_weights_.data() + eigen_weights_.size());
            vector<Particle, Eigen::aligned_allocator<Particle>> resampParticles;

            resampParticles.reserve(N_);
            int idx;

            for (unsigned int i = 0; i < N_; i++)
            {
                idx = d(rand_engine_);
                resampParticles.push_back(particles_[idx]);
                resampParticles[i].parent_id=idx;

            }
            //std::sort(resampParticles.data(), resampParticles.data() + resampParticles.size(),[](Particle lhs, Particle rhs){return rhs.weight > lhs.weight;});
            particles_ = std::move(resampParticles);
            // debug_.safeParticleWeights(2);
        }
    }
}

void FastSlam::LVResample(){

    //std::cout << "entered Low Variance Resample" << std::endl;

     std::uniform_real_distribution<> distr(0, 1.0 / N_);

     vector<Particle, Eigen::aligned_allocator<Particle>> resampParticles;
     resampParticles.reserve(N_);

     int i=0;
     double c=particles_[i].weight;
     double r=distr(rand_engine_);
     for(int j=0; j<N_; j++){
        double U=r+j*(1.0 / N_);
        while (U>c){
            i++;
            c+=particles_[i].weight;
        }
        resampParticles.push_back(particles_[i]);
        resampParticles[j].parent_id=i;
        resampParticles[j].id=j;

    }
    particles_ = std::move(resampParticles);

    // debug_.safeParticleWeights(1);
}

//initialize_location for live execution
//Erik: Function to localize the car at the start of the track? The function is never used.
void FastSlam::initialize_location(Particle &p, const custom_msgs::ConeArrayConstPtr &temp_cloud) const
{
	int obs_left, obs_right;
	int lm_left, lm_right;
	double range_obs_left = 10.0, range_obs_right = 10.0, range_lm_left = 10.0, range_lm_right = 10.0;

	std::cout << "init" << std::endl;
	std::cout << "size: " << temp_cloud->cones.size() << std::endl;
	//find observations and landmarks that are closest to the car
	for (int i = 0; i < temp_cloud->cones.size(); ++i)
	{
		std::cout << "cone: " << i << std::endl;
		//getting observation
		double temp_range = std::hypot(temp_cloud->cones[i].position.x, temp_cloud->cones[i].position.y);
		if (temp_range < range_obs_left && temp_cloud->cones[i].position.y > 0) //Erik: why y position? Isnt y the direction we drive in. So x>0 should be the right and x<0 the left cone.
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
		if (temp_range < range_lm_left && p.landmarks[i].mu(1) > 0)// Erik:Why mu(1)? Shouldnt it be mu(0) so the x direction
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

    //Erik: forming the normalized vector (direction) to the left and right observations
	Eigen::Vector2d n_left((temp_cloud->cones[obs_left].position.x / range_obs_left), (temp_cloud->cones[obs_left].position.y / range_obs_left));
	Eigen::Vector2d n_right((temp_cloud->cones[obs_right].position.x / range_obs_right), (temp_cloud->cones[obs_right].position.y / range_obs_right));
	
    //Erik: creating two points right next to the right and left landmarks extending in the direction of the observations
	Eigen::Vector2d p_end_left((p.landmarks[lm_left].mu(0) + n_left(0)), (p.landmarks[lm_left].mu(1) + n_left(1)));
	Eigen::Vector2d p_end_right((p.landmarks[lm_right].mu(0) + n_right(0)), (p.landmarks[lm_right].mu(1) + n_right(1)));
	
    //Erik: creating two lines that intersect in the position of the LIDAR
	Eigen::Hyperplane<double, 2> left = Eigen::Hyperplane<double, 2>::Through(p.landmarks[lm_left].mu, p_end_left);
	Eigen::Hyperplane<double, 2> right = Eigen::Hyperplane<double, 2>::Through(p.landmarks[lm_right].mu, p_end_right);
	std::cout << "Intersection:\n"<< left.intersection(right) << '\n';
}

//return the index of the best particle, meaning the particle with the highest weight.
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
    return (sqrt(pow(landmark.sigma(0, 0), 2) + pow(landmark.sigma(1, 1), 2)) < cone_covariance_threshold_);
}



//for our use case the sample size is always 1 and and the resulting sampledPose is just a vector not a matrix. If the samplesize is >1 you would need to change Eigen::Vector3d to Eigen::MatrixXd
void FastSlam::multivariate_normal(Eigen::MatrixXd &sampledPose ,Eigen::Vector3d &mean,Eigen::Matrix3d &covariance,int samplesize){

bool usingChol; //flag to see whether cholesky method was used or eigenvalue decomposition.

//static std::random_device my_rd{};
//static std::mt19937 my_rand_engine{my_rd()};
std::normal_distribution<double> my_normaldist(0,1); //this must be a standard normal distribution

auto glambda = [&](){return my_normaldist(rand_engine_);};
Eigen::MatrixXd random_mat;
random_mat = Eigen::MatrixXd::NullaryExpr(mean.rows(),samplesize,glambda); //creates an Matrix with normally distributed random numbers

Eigen::Matrix3d petturbedCovariance=covariance+0.001*Eigen::Matrix3d::Identity(); //add some error to the covariance matrix for numerical stability

Eigen::LLT<Eigen::Matrix3d> lltOfcov(petturbedCovariance); // compute the Cholesky decomposition

Eigen::Matrix3d normTransform;

  if (lltOfcov.info()==Eigen::Success) { //check if cholesky method worked
    normTransform = lltOfcov.matrixL();
    usingChol=true;
  } else {  //if it didnt work, use eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance);
    normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    usingChol=false;
  }

sampledPose = (normTransform * random_mat).colwise()+ mean;

// debug_.testMultivariate(sampledPose, mean, covariance, random_mat,normTransform, usingChol); //uncomment if you want to test
}






