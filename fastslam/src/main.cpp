#include "common.h"
#include "fastslam.h"
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Quaternion.h"
#include <nav_msgs/Odometry.h>
#include <fssim_common/State.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <chrono>
#include <iostream>
#include <fstream>

using namespace std;

ros::Publisher marker_pub1, marker_pub2, marker_pub3, marker_pub4, marker_pub5,
    odom_pub;
geometry_msgs::Point p;
FastSlam fastslam;
visualization_msgs::Marker points1, points2, arrow;
std::ofstream myfile;
nav_msgs::OdometryConstPtr last_state;
std::vector<double> time_vector;

void intializing_visualizations() {
	points1.header.frame_id = points2.header.frame_id = arrow.header.frame_id =
	    "/map";
	points1.header.stamp = points2.header.stamp = arrow.header.stamp =
	    ros::Time::now();
	points1.ns = points2.ns = arrow.ns = "points_and_lines";
	points1.action = points2.action = arrow.action =
	    visualization_msgs::Marker::ADD;
	points1.pose.orientation.w = points2.pose.orientation.w = 1.0;
	points1.id = 0;
	points2.id = 1;
	arrow.id = 2;
	points1.type = points2.type = visualization_msgs::Marker::POINTS;
	arrow.type = visualization_msgs::Marker::ARROW;
	points1.scale.x = points2.scale.x = 0.3;
	points1.scale.y = points2.scale.y = 0.3;
	points1.color.b = points2.color.r = 1.0f;
	points1.color.g = points2.color.g = 0;
	points1.color.r = points2.color.b = 0;
	points1.color.a = points2.color.a = 1.0;
	arrow.scale.x = 4.0;
	arrow.scale.y = 0.5;
	arrow.scale.z = 0.5;
	arrow.color.g = 1.0f;
	arrow.color.a = 1.0;
	arrow.color.r = 0.0f;
	arrow.color.b = 0.0f;
}

void callback(const custom_msgs::ConeArrayConstPtr& z,
              const nav_msgs::OdometryConstPtr& state) {
	// transforming pointcloud2 to pcl format
	Eigen::VectorXd bestParticle;

	std::chrono::steady_clock::time_point begin =
	    std::chrono::steady_clock::now();

	// prediction and correction steps
	fastslam.Prediction(state, last_state);
	fastslam.Correction(z);
	last_state = state;

	geometry_msgs::Point pt1;
	for (int i = 0; i < fastslam.particles.size(); i++) {
		pt1.x = fastslam.particles[i].pose(0);
		pt1.y = fastslam.particles[i].pose(1);
		pt1.z = 0;
		points1.points.push_back(pt1);
	}

	// Resampling
	fastslam.Resample();
	fastslam.getBestPoseAndLandmark(bestParticle);

	std::cout << "landmarks: " << (bestParticle.size() - 3.0) / 2.0
	          << std::endl;
	for (int i = 0; i < (bestParticle.size() - 3.0) / 2.0; i++) {
		pt1.x = bestParticle(2 * i + 3);
		pt1.y = bestParticle(2 * i + 4);
		points2.points.push_back(pt1);
	}

	// laps counter
	if (std::hypot(bestParticle(0), bestParticle(1)) < 1.0 &&
		std::chrono::duration_cast<std::chrono::seconds>(
	        begin - fastslam.lap_counter_pause)
	            .count() > 10.0) {
		fastslam.lap++;
		fastslam.lap_counter_pause = std::chrono::steady_clock::now();
	}

	static tf::TransformBroadcaster br;
	tf::Transform transform;

	transform.setOrigin(tf::Vector3(bestParticle(0), bestParticle(1), 0.0));
	tf::Quaternion q;
  geometry_msgs::Quaternion quat_msg;
	q.setRPY(0, 0, bestParticle(2));
	transform.setRotation(q);
	quaternionTFToMsg(q, quat_msg);
	br.sendTransform(
	    tf::StampedTransform(transform, ros::Time::now(), "/map", "/pandar"));

	nav_msgs::Odometry odom;	

	odom.header.stamp = state->header.stamp;
	odom.header.frame_id = "map";
  
	// set the position
	odom.pose.pose.position.x = bestParticle(0);
	odom.pose.pose.position.y = bestParticle(1);
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = quat_msg;
	odom.child_frame_id = "pandar";
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = 0;

	// publish the message
	odom_pub.publish(odom);
	marker_pub1.publish(points1);
	marker_pub2.publish(points2);

	points2.points.clear();
	points1.points.clear();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	double d_time = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000.0;
	if(fastslam.lap > 1){
		time_vector.push_back(d_time);		
	}
	std::cout << "lap: " << fastslam.lap << std::endl;
	std::cout << "Time elapsed = " << d_time << std::endl;
	std::cout << "average = " << std::accumulate(time_vector.begin(), time_vector.end(), 0.0)/time_vector.size()<< std::endl;
	std::cout << "-----------------------------" << std::endl;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "fastslam");
	ros::NodeHandle n("~");
	intializing_visualizations();

	marker_pub1 = n.advertise<visualization_msgs::Marker>("/odometry", 1);
	marker_pub2 = n.advertise<visualization_msgs::Marker>("/landmarks", 1);
	marker_pub3 = n.advertise<visualization_msgs::Marker>("/orientation", 1);
	marker_pub4 = n.advertise<visualization_msgs::Marker>("/matches", 1);
	marker_pub5 = n.advertise<visualization_msgs::Marker>("/false_matches", 1);
	odom_pub = n.advertise<nav_msgs::Odometry>("/fs_odom", 1);

	fastslam.Initialize(n);
	int queue_size = n.param<int>("queue_size", 80);
	std::cout << "que " << queue_size << std::endl;
	last_state = nullptr;

	// message_filters::Subscriber<sensor_msgs::PointCloud2> cones_sub(n,
	// "/lidar/cones", 1);
	message_filters::Subscriber<custom_msgs::ConeArray> cones_sub(
	    n, "/perception/lidar/cones", 1);
	message_filters::Subscriber<nav_msgs::Odometry> state_sub(n, "/gps/odom",
	                                                          1);
	// ros::Subscriber sub = n.subscribe("fssim/base_pose_ground_truth", 1,
	// chatterCallback);
	typedef message_filters::sync_policies::ApproximateTime<
	    custom_msgs::ConeArray, nav_msgs::Odometry>
	    MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(queue_size),
	                                                 cones_sub, state_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	std::cout << "waiting for callbacks" << std::endl;

	ros::spin();

	return 0;
}
