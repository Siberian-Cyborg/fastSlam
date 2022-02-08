#include "common.h"
#include "fastslam.h"
#include <iomanip>
#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <custom_msgs/WheelValues.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <custom_msgs/GlobalMap.h>
#include <custom_msgs/Cone.h>
#include <custom_msgs/ConeArray.h>
#include <custom_msgs/ConeWithCovariance.h>
#include <custom_msgs/ConeWithCovarianceArray.h>
#include <std_msgs/Float64.h>
#include "profc.h"
#include <mutex>

# define M_PI  3.14159265358979323846 

ros::Publisher marker_pub1, marker_pub2, marker_pub3, marker_pub4, odom_pub, map_pub, particle_pub, gps_particle_pub;
ros::Publisher map_pub2, bestProb_pub, logging_pub;
geometry_msgs::Point particle_pose;
FastSlam fastslam;
visualization_msgs::Marker points1, points2, points3,points4;
//sensor_msgs::ImuConstPtr last_state;
nav_msgs::Odometry last_state;
std::vector<double> time_vector;
bool initialized;
bool mapped=false;
bool left_start=false;
bool timed=false;
custom_msgs::GlobalMap map_;
tf::Transform initial_transform;
unique_ptr<message_filters::Cache<nav_msgs::Odometry>> cache;

std::once_flag flag1;

void intializing_visualizations()
{
	points1.header.frame_id = points2.header.frame_id = points3.header.frame_id = "/map";
	points4.header.frame_id = "/map";
	points1.header.stamp = points2.header.stamp = points3.header.stamp = points4.header.stamp = ros::Time::now();
	points1.ns = points2.ns = points3.ns = points4.ns = "points_and_lines";
	points1.action = points2.action = points3.action = points4.action = visualization_msgs::Marker::ADD;
	points1.pose.orientation.w = points2.pose.orientation.w = points3.pose.orientation.w = points4.pose.orientation.w = 1.0;

	points1.id = 0;
	points2.id = 1;
	points3.id = 2;
	points4.id = 3;

	points1.type = points2.type = points3.type = points4.type = visualization_msgs::Marker::POINTS;

	points1.scale.x = points2.scale.x = points3.scale.x = points4.scale.x = 0.3;
	points1.scale.y = points2.scale.y = points3.scale.y = points4.scale.y = 0.3;

	points1.color.r = 0; //particles
	points1.color.g = 1.0;
	points1.color.b = 1.0;

	points2.color.r = 0; //left
	points2.color.g = 0;
	points2.color.b = 1.0;

	points3.color.r = 1.0; //right
	points3.color.g = 1.0;
	points3.color.b = 0;

	points4.color.r = 1.0; //landmarks
	points4.color.g = 0.0;
	points4.color.b = 0.0;

	points1.color.a = points2.color.a = points3.color.a = points4.color.a = 1.0;
}


void getParticleAndLandmarkPositions(){

	geometry_msgs::Point particle_pose;
	for (const auto& particle : fastslam.particles_)
	{
		particle_pose.x = particle.pose(0);
		particle_pose.y = particle.pose(1);
		particle_pose.z = 0;
		points1.points.push_back(particle_pose); //position of all particles
	}
	// get best particle
	geometry_msgs::Point pt1;
	int index = fastslam.MaxIndex();
	Particle bestParticle= fastslam.particles_[index];
	if (index == fastslam.N_ -1 ) {
		//std::cout << "last particle" << std::endl;  //the best particle is the gps particle
	}else{
	    //std::cout << "not last particle" << std::endl;
	}
	for (int i = 0; i < bestParticle.landmarks.size(); i++)
	{
		pt1.x = bestParticle.landmarks[i].mu(0);
		pt1.y = bestParticle.landmarks[i].mu(1);
		points4.points.push_back(pt1); //all cone positions of best particle

		//this is not currently working, because no color is given by perception
		if (bestParticle.landmarks[i].type == 0) //left cones of best particle
		{
			points2.points.push_back(pt1);
		}
		else if (bestParticle.landmarks[i].type == 1) //right cones of best particle
		{
			points3.points.push_back(pt1);
		}
	}

}

void publishBestCurrentMap(const custom_msgs::ConeArrayConstPtr &z , const Particle& bestParticle){

	custom_msgs::ConeWithCovarianceArray cone_cov_array;
	cone_cov_array.header.stamp = z->header.stamp;
	cone_cov_array.header.frame_id = "map";
	custom_msgs::ConeWithCovariance cone_cov;
	for (int i = 0; i < bestParticle.landmarks.size(); i++)
	{
	    if (fastslam.isConeBelowThreshold(bestParticle.landmarks[i]))
	    {
            cone_cov.position.x = bestParticle.landmarks[i].mu(0);
            cone_cov.position.y = bestParticle.landmarks[i].mu(1);
            cone_cov.position.z = 0;
            cone_cov.covariance[0] = bestParticle.landmarks[i].sigma(0,0);
            cone_cov.covariance[1] = bestParticle.landmarks[i].sigma(0,1);
            cone_cov.covariance[2] = bestParticle.landmarks[i].sigma(1,0);
            cone_cov.covariance[3] = bestParticle.landmarks[i].sigma(1,1);
            cone_cov.type = custom_msgs::ConeWithCovariance::OTHER;
            cone_cov_array.cones.push_back(cone_cov);
	    }
	}
	map_pub2.publish(cone_cov_array);
}


void publishFinalMap(custom_msgs::GlobalMap &map, custom_msgs::WheelValuesConstPtr last_state, const Particle& bestParticle, bool &mapped){


    std::cout<<"entered publishFinalMap"<<std::endl;
	map.header.frame_id = "/map";
	double someDouble= fastslam.p_new_landmark_;
	map.header.stamp = last_state->header.stamp;
	tf::Transform transform;
	custom_msgs::Cone cone;
	/*
	std::string homepath = getenv("HOME");
	std::ofstream myfile;
  	myfile.open ((homepath + "/map_live.txt").c_str());
	for (int i = 0; i < bestParticle.landmarks.size(); i++)
	{
		cone.position.x = bestParticle.landmarks[i].mu(0);
		cone.position.y = bestParticle.landmarks[i].mu(1);
		myfile << "id"<<": "<<i<<"\n";
		myfile <<bestParticle.landmarks[i].mu(0)<<"\n";
		myfile <<bestParticle.landmarks[i].mu(1)<<"\n";
		myfile <<bestParticle.landmarks[i].type<<"\n";
		myfile << "----------------------------\n";
		map.cones.push_back(cone); // adds the position of the cones of the best particle to the map
	}
	myfile.close();
	*/
	tf::Quaternion q;
	geometry_msgs::Quaternion quat_msg;
	geometry_msgs::Pose quat_pose;
	for (int i = 0; i < bestParticle.history.size(); i++)
	{
		quat_pose.position.x = bestParticle.history[i](0);
		quat_pose.position.y = bestParticle.history[i](1);
		q.setRPY(0, 0, bestParticle.history[i](2));
		transform.setRotation(q);
		quaternionTFToMsg(q, quat_msg);
		quat_pose.orientation = quat_msg;
		map.poses.push_back(quat_pose); // adds all pose of the best particle to the map
	}

    /*
    std::ofstream myfile2;
  	myfile2.open ("/home/krauter/landmarks.txt");
	myfile2 << bestParticle.landmarks.size()<< "\n"; //writing the number of particles to a file for debugging
	myfile2.close();
	*/
	std::cout<<"publishing map !"<<std::endl;
	map_pub.publish(map_);
	mapped = true;
	std::cout << "------------------------" << std::endl;

}

void safeMapToFile(const Particle& bestParticle){

	std::string homepath = getenv("HOME");
	std::string name = (homepath + "/db020/modules_linux/03_stateestimation/map.txt").c_str();
	std::ofstream myfile;
	//if file does not yet exist create it and write the map in it.
	std:ifstream f(name);
    if(!f.good()){
        ROS_WARN_STREAM("Writing the map to file!");
        myfile.open((homepath + "/db020/modules_linux/03_stateestimation/map.txt").c_str());
        for (int i = 0; i < bestParticle.landmarks.size(); i++)
        {
            if (fastslam.isConeBelowThreshold(bestParticle.landmarks[i])){
                myfile << "id"<<": "<<i<<"\n";
                myfile <<bestParticle.landmarks[i].mu(0)<<"\n";
                myfile <<bestParticle.landmarks[i].mu(1)<<"\n";
                myfile << "----------------------------\n";
            }
        }
        myfile.close();

    }
    fastslam.mapped_ = true;
}

void publishParticlesAndLandmarks(const custom_msgs::ConeArrayConstPtr &z, const Particle& bestParticle) {

	tf::Transform transform;
	static tf::TransformBroadcaster br;
	transform.setOrigin(tf::Vector3(bestParticle.pose(0), bestParticle.pose(1), 0.0));
	tf::Quaternion q;
	geometry_msgs::Quaternion quat_msg;
	q.setRPY(0, 0, bestParticle.pose(2));
	transform.setRotation(q);
	quaternionTFToMsg(q, quat_msg);
	br.sendTransform(tf::StampedTransform(transform, z->header.stamp, "/map", "/os1_sensor"));

	nav_msgs::Odometry odom;
	odom.header.stamp = z->header.stamp;
	odom.header.frame_id = "map";
	// set the position
	odom.pose.pose.position.x = bestParticle.pose(0);
	odom.pose.pose.position.y = bestParticle.pose(1);
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = quat_msg;
	odom.child_frame_id = "pandar";
	odom.twist.twist.linear.x = bestParticle.vel(0);
	odom.twist.twist.linear.y = bestParticle.vel(1);
	odom.twist.twist.angular.z = 0.0; //bestParticle.vel(2)
	// publish the message
	odom_pub.publish(odom);       //publish position and velocity of best particle
	/*
	marker_pub1.publish(points1); //publish position of all particles
	marker_pub2.publish(points2); //publish position of left cones of best particle
	marker_pub3.publish(points3); //publish position of right cones of best particle
	marker_pub4.publish(points4); //publish position of all cones
	points2.points.clear();
	points4.points.clear();
	points3.points.clear();
	points1.points.clear();
	*/
}


void predictionCallback(const nav_msgs::Odometry &state) {
	PROFC_NODE("predcallback");
	boost::shared_ptr<const nav_msgs::Odometry> vel = cache->getElemBeforeTime(state.header.stamp);
	fastslam.Prediction2(&state, &last_state, vel);
	last_state = state;
}

void correctionCallback(const custom_msgs::ConeArrayConstPtr &z) {
	PROFC_NODE("correccallback");
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    //custom_msgs::LogArray log_msg;
	//Correction Step
	fastslam.Correction(z);

    //prepare poses of particles and cones of best particle for publishing
    //std::cout << "calling getParticleAndLandmarkPositions" << std::endl;
	//getParticleAndLandmarkPositions();

    //get best particle and publish its cones
    //std::cout << "calling fastslam.MaxIndex" << std::endl;
	int index = fastslam.MaxIndex();
	Particle bestParticle = fastslam.particles_[index];
	publishBestCurrentMap(z, bestParticle);
	/*
    // TODO: do we need this with new logging?
	bestProb_pub.publish(log (bestParticle.weight));

	custom_msgs::LogValue best_particle_prob;
    best_particle_prob.key = "best_particle_prob";
    best_particle_prob.value = log(bestParticle.weight);
	log_msg.data.push_back(best_particle_prob);
    */
    // Resampling step
	fastslam.Resample();

	// laps counter
    if (std::hypot(bestParticle.pose(0), bestParticle.pose(1)) > 15.0 && !left_start)
        {
                left_start = true;
        }

	if (std::hypot(bestParticle.pose(0), bestParticle.pose(1)) < 2.0 && std::chrono::duration_cast<std::chrono::seconds>(begin - fastslam.lap_counter_pause_).count() > 10.0 && left_start)
	{

		fastslam.lap_++;
		std::cout<<"Loop Closed (FastSlam)"<<std::endl;
		std::cout<<"lap counter: "<< fastslam.lap_ <<std::endl;
		fastslam.lap_counter_pause_ = std::chrono::steady_clock::now();
		left_start = false;
	}

    //After loop closure the final map ist saved and published
	if ((fastslam.lap_ > 0 || fastslam.found_loop_closure_) && !fastslam.mapped_)
	{
	    //publishFinalMap(map_, last_state2, bestParticle, mapped); //sets mapped=true and writes cones into map_
	    safeMapToFile(bestParticle);
	}

    //std::cout << "calling publishParticlesAndLandmarks" << std::endl;
	publishParticlesAndLandmarks(z, bestParticle);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double d_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;

	if (d_time > 100) {
		ROS_WARN_STREAM("High latency: " << d_time << "ms! High risk of dropped frames");
	}

}

void closedloopCallback(const custom_msgs::GlobalMap &map){
    if(!timed){
        fastslam.loop_closure_timer_start_ = std::chrono::steady_clock::now();
        timed = true;
    }
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = current_time-fastslam.loop_closure_timer_start_;
    if(elapsed_seconds.count() > fastslam.wait_time_after_closure_){
        fastslam.found_loop_closure_ = true;
        std::cout<<"Global Path Found (FastSlam)"<<std::endl;
    }
}

 void callback(const custom_msgs::ConeArrayConstPtr &z, const nav_msgs::OdometryConstPtr &state){
//    ROS_WARN_STREAM("calllbaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaack");
    predictionCallback(*state);
    correctionCallback(z);
 }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fastslam");
	ros::NodeHandle n("~");
	intializing_visualizations();
	bool sim = n.param<bool>("amz_sim", false);
	std::cout << "amz_sim: " << sim << std::endl;

	marker_pub1 = n.advertise<visualization_msgs::Marker>("/odometry", 1);
	marker_pub2 = n.advertise<visualization_msgs::Marker>("/left_landmarks", 1);
	marker_pub3 = n.advertise<visualization_msgs::Marker>("/right_landmarks", 1);
	marker_pub4 = n.advertise<visualization_msgs::Marker>("/landmarks", 1);
	map_pub2 = n.advertise<custom_msgs::ConeWithCovarianceArray>("/slam/map", 1); //map of best particle
	map_pub = n.advertise<custom_msgs::GlobalMap>("/perception/ClosedLoop", 1);
	odom_pub = n.advertise<nav_msgs::Odometry>("/fastslam/odometry", 1); //publish position and velocity of best particle
	particle_pub = n.advertise<geometry_msgs::PoseArray>("/particles", 1);
	gps_particle_pub = n.advertise<geometry_msgs::PoseStamped>("/gps/particle", 1);
	//bestProb_pub = n.advertise<std_msgs::Float64>("/propability/best/particle", 1);
	//logging_pub = n.advertise<custom_msgs::LogArray>("/slam/logging", 1);

    std::cout << "calling Initialize" << std::endl;
	fastslam.Initialize(n);

	fastslam.lap_counter_pause_ = std::chrono::steady_clock::now();
	int queue_size = n.param<int>("queue_size", 80);
	std::cout << "Number of Particles: " << FastSlam::N_ << std::endl;
	std::cout<<"Limit for effective number of Particles: "<< fastslam.Neff_limit_<<std::endl;

	//last_state = nullptr;
	message_filters::Subscriber<nav_msgs::Odometry> gps_sub(n, "/gps/odom", 1 , ros::TransportHints().tcpNoDelay()); //subscribing to gps data from oxford solutions device
	cache = std::make_unique<message_filters::Cache<nav_msgs::Odometry>>(gps_sub, 100);
    /*
	bool decoupled = true;
	if(decoupled)
	{
	    ros::Subscriber sub_odom = n.subscribe("/EKF/odometry", 1, predictionCallback, ros::TransportHints().tcpNoDelay());
        //ros::Subscriber sub_odom_wheelrev = n.subscribe("/stateestimation/wheel_revs_actual", 1, predictionCallback2, ros::TransportHints().tcpNoDelay());
        ros::Subscriber sub_cones = n.subscribe("/perception/lidar/local_cones", queue_size, correctionCallback, ros::TransportHints().tcpNoDelay());
        ros::Subscriber sub_closed_loop = n.subscribe("/planning/closedloop", 1, closedloopCallback, ros::TransportHints().tcpNoDelay());
        std::cout << "waiting for callbacks" << std::endl;
        ros::spin();
	}
	else
	{
		throw std::invalid_argument("decoupled and inertial not supported");
	}
	return 0;
	*/
	bool decoupled = false;
	if(decoupled)
	{
		//ros::Subscriber sub_odom = n.subscribe("/os1_cloud_node/imu", 1, predictionCallback, ros::TransportHints().tcpNoDelay());
		ros::Subscriber sub_odom = n.subscribe("/EKF/odometry", 1, predictionCallback, ros::TransportHints().tcpNoDelay());
		ros::Subscriber sub_cones = n.subscribe("/perception/lidar/local_cones", 1, correctionCallback, ros::TransportHints().tcpNoDelay());
		ros::Subscriber sub_closed_loop = n.subscribe("/planning/closedloop", 1, closedloopCallback, ros::TransportHints().tcpNoDelay());
		std::cout << "waiting for callbacks" << std::endl;
		ros::spin();
	}
	else
	{
		message_filters::Subscriber<custom_msgs::ConeArray> cones_sub(n, "/perception/lidar/local_cones", 10);
		message_filters::Subscriber<nav_msgs::Odometry> Odometry_sub(n, "/EKF/odometry", 400);
		ros::Subscriber sub_closed_loop = n.subscribe("/planning/closedloop", 1, closedloopCallback, ros::TransportHints().tcpNoDelay());
		typedef message_filters::sync_policies::ApproximateTime<custom_msgs::ConeArray, nav_msgs::Odometry> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(400), cones_sub, Odometry_sub);
		sync.registerCallback(boost::bind(&callback, _1, _2));
		std::cout << "waiting for callbacks" << std::endl;
		ros::spin();
	}
	return 0;
}
