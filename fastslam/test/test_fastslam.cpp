#include <gtest/gtest.h>

#include "fastslam.h"
#include "tools.h"

TEST(Fastslam, dummy_test) {
	FastSlam fastslam;
	EXPECT_FALSE(fastslam.is_initialized_);
}

TEST(Fastslam, normalizeangle) {
	Tools tools;
	double angle;
	angle = tools.normalize_angle(2*3.14+0.01);
	ASSERT_LE(angle,3.14);
	std::cout << "Angle 2*pi + eps goes to:" << angle << std::endl;
}

TEST(Fastslam, bearingangle) {
	Tools tools;
	double angle;
	// y = 1, x = 0
	// straight infront of lidar
	int y = 1;
	int x = 0;
	angle = tools.normalize_angle(std::atan2(y, x));
	std::cout << "Angle between:" << x << " , " << y << ": " << angle << std::endl;
	// expect 90 deg
	ASSERT_NEAR(angle,1.5707, 1e-04); // pi/2
	// y = 0, x = 1
	// straight to the right of the lidar
	y = 0;
	x = 1;
	angle = tools.normalize_angle(std::atan2(y, x));
	// expect 0 deg
	ASSERT_FLOAT_EQ(angle,0.0);
	std::cout << "Angle between:" << x << " , " << y << ": " << angle << std::endl;
}

TEST(Fastslam, associate_observation) {
	FastSlam fastslam;
	Tools tools;
	fastslam.Q_ = Eigen::MatrixXd::Zero(2, 2);
	fastslam.max_matching_distance = 15;
 
	Particle p;
	p.pose = {0,0,0};

	std::cout << "pose;"<< p.pose << std::endl;
	// cone #1 at 1;1
	LandMark l1;
	l1.mu = {1,1};
	l1.sigma = Eigen::MatrixXd::Zero(2,2);
	l1.sigma(0, 0) = 0.001;
	l1.sigma(1, 1) = 0.001;
	l1.id = 1;
	l1.type = 3;

	// cone #2 at 10;10
	LandMark l2;
	l2.mu = {10,10};
	l2.sigma = Eigen::MatrixXd::Zero(2,2);
	l2.sigma(0, 0) = 0.001;
	l2.sigma(1, 1) = 0.001;
	l2.id = 2;
	l2.type = 3;
	
	p.landmarks.push_back(l2);
	p.landmarks.push_back(l1);

	Eigen::MatrixXd best_sig, best_Q, best_K, best_H;
	Eigen::Vector2d best_z_diff;
	int l; 
	double best_prob = 0;
	double nn;
	
	// new detected cone at 1;1
	// should be 100% associated with #1
	RadarReading z;
	auto x = 1.0;
	auto y = 1.0;
	z.range = std::sqrt(x*x + y*y);
	z.bearing = tools.normalize_angle(std::atan2(y,x));
	// gx and gy is 1.0 and 1.0
	fastslam.Associate_observation(p, 1.0 , 1.0, best_sig, best_Q, best_K, best_H, best_z_diff,
	z, nn, best_prob, l, 3);
	std::cout << "Best prob: " << best_prob << std::endl;
	std::cout << "Best Q: \n" << best_Q << std::endl;
	std::cout << "Best z_diff: \n" << best_z_diff << std::endl;
	std::cout << "Best id: " << l << std::endl;
	// expect association to the first landmark
	EXPECT_EQ(l, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

