#include "dynamical_systems/Linear.hpp"
#include <gtest/gtest.h>
#include <zmq.hpp>
#include <unistd.h>


TEST(EvaluateDynamicalSystemPositionOnly, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);
	StateRepresentation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
	StateRepresentation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random());
	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.001);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);	

}

TEST(EvaluateDynamicalSystemOrientationOnly, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);

	StateRepresentation::CartesianPose current_pose("robot", Eigen::Vector3d(0,0,0));
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("robot", Eigen::Vector3d(0,0,0), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	std::cout << current_pose << std::endl;
	std::cout << target_pose << std::endl;
	std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);
}

TEST(EvaluateDynamicalSystem, PositiveNos)
{
	DynamicalSystems::Linear<StateRepresentation::CartesianState> linearDS(1);

	StateRepresentation::CartesianPose current_pose("robot", 10 * Eigen::Vector3d::Random());
	Eigen::Array4d orientation = Eigen::Array4d::Random();
	StateRepresentation::CartesianPose target_pose("robot", 10 * Eigen::Vector3d::Random(), Eigen::Quaterniond(orientation(0), orientation(1), orientation(2), orientation(3)));

	linearDS.set_attractor(target_pose);

	unsigned int nb_steps = 100;
	double dt = 0.1;

	for(unsigned int i=0; i<nb_steps; ++i)
	{
		StateRepresentation::CartesianVelocity velocity = linearDS.evaluate(current_pose);
		current_pose += dt * velocity;
	}

	std::cout << current_pose << std::endl;
	std::cout << target_pose << std::endl;
	std::cout << abs(current_pose.get_orientation().dot(target_pose.get_orientation())) << std::endl;

	for(int i=0; i<3; ++i) EXPECT_NEAR(current_pose.get_position()(i), target_pose.get_position()(i), 0.01);
	EXPECT_TRUE(abs(current_pose.get_orientation().dot(target_pose.get_orientation())) > 1-10E-4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}