#include "state_representation/Cartesian/CartesianPose.hpp"
#include "state_representation/Cartesian/CartesianTwist.hpp"
#include "state_representation/Cartesian/CartesianWrench.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <zmq.hpp>
#include <unistd.h>

Eigen::Vector4d quat_mul(Eigen::Vector4d a, Eigen::Vector4d b)
{
	Eigen::Vector4d res;
	res[0] =  a[1]*b[2] - a[2]*b[1] + a[0]*b[3] + a[3]*b[0];
	res[1] =  a[2]*b[0] - a[0]*b[2] + a[1]*b[3] + a[3]*b[1];
	res[2] =  a[0]*b[1] - a[1]*b[0] + a[2]*b[3] + a[3]*b[2];
	res[3] = -a[0]*b[0] - a[1]*b[1] - a[2]*b[2] + a[3]*b[3];
	if (res[3]<0)
		res *= -1;
	return res;
}

TEST(NegateQuaternion, PositiveNos)
{
	Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
	Eigen::Quaterniond q2 = Eigen::Quaterniond(-q.coeffs());

	EXPECT_TRUE(q.w() == -q2.w());
	for (int i = 0; i < 3; ++i) EXPECT_TRUE(q.vec()(i) == -q2.vec()(i));
}

TEST(QuaternionMultiply, PositiveNos)
{
	Eigen::Quaterniond q1 = Eigen::Quaterniond::UnitRandom();
	Eigen::Quaterniond q2 = Eigen::Quaterniond::UnitRandom();
	Eigen::Quaterniond qres = q1 * q2;
	if(qres.dot(q1) < 0) qres = Eigen::Quaterniond(-qres.coeffs());

	std::cout << qres.coeffs() << std::endl;
	Eigen::Vector4d res = quat_mul(q1.coeffs(), q2.coeffs());
	std::cout << res << std::endl;

	for (int i = 0; i < 4; ++i) EXPECT_TRUE(qres.coeffs()(i) == res(i));

	Eigen::Quaterniond q3 = Eigen::Quaterniond(-q2.coeffs());
	qres = q1 * q3;

	std::cout << qres.coeffs() << std::endl;
	res = quat_mul(q1.coeffs(), q3.coeffs());
	std::cout << res << std::endl;

	for (int i = 0; i < 4; ++i) EXPECT_TRUE(qres.coeffs()(i) == res(i));

}

TEST(MultiplyTransformsBothOperators, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1,0,0,0);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(1,0,0,0);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	StateRepresentation::CartesianPose tf3 = tf1 * tf2;
	tf1 *= tf2;
	
	EXPECT_EQ(tf3.get_name(), "t2");
	for(int i=0; i<tf1.get_position().size(); ++i) EXPECT_NEAR(tf1.get_position()(i), tf3.get_position()(i), 0.00001);
}


TEST(MultiplyTransformsSameOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1,0,0,0);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(1,0,0,0);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	tf1 *= tf2;
	
	Eigen::Vector3d pos_truth(5,7,9);
	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
}

TEST(MultiplyTransformsDifferentOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(4,5,6);
	Eigen::Quaterniond rot2(0., 0., 0.70710678, 0.70710678);
	StateRepresentation::CartesianPose tf2("t2", pos2, rot2, "t1");

	tf1 *= tf2;
	
	Eigen::Vector3d pos_truth(5,-4,8);
	Eigen::Quaterniond rot_truth(0.,0.,0.,1.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1-10E-4);
}

TEST(TestInverseNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(1., 0., 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 = tf1.inverse();
	
	Eigen::Vector3d pos_truth(-1,-2,-3);
	Eigen::Quaterniond rot_truth(1.,0.,0.,0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	EXPECT_EQ(tf1.get_name(), "world");
	EXPECT_EQ(tf1.get_reference_frame(), "t1");
	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1-10E-4);
}

TEST(TestInverseNonNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 = tf1.inverse();
	
	Eigen::Vector3d pos_truth(-1,-3,2);
	Eigen::Quaterniond rot_truth(0.70710678, -0.70710678, 0., 0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1-10E-4);
}

TEST(TestMultiplyInverseNonNullOrientation, PositiveNos)
{
	Eigen::Vector3d pos1(1,2,3);
	Eigen::Quaterniond rot1(0.70710678, 0.70710678, 0., 0.);
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	tf1 *= tf1.inverse();
	
	Eigen::Vector3d pos_truth(0,0,0);
	Eigen::Quaterniond rot_truth(1., 0., 0., 0.);

	std::cerr << "position" << std::endl;
	std::cerr << tf1.get_position() << std::endl;
	std::cerr << "orientation" << std::endl;
	std::cerr << tf1.get_orientation().coeffs() << std::endl;

	for(int i=0; i<pos_truth.size(); ++i) EXPECT_NEAR(tf1.get_position()(i), pos_truth(i), 0.00001);
    EXPECT_TRUE(abs(tf1.get_orientation().dot(rot_truth)) > 1-10E-4);
}

TEST(TestAddTwoPoses, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(1,0,0);
	Eigen::Quaterniond rot2(0,1,0,0); 
	StateRepresentation::CartesianPose tf2("t1", pos2, rot2);

	std::cout << tf1 + tf2 << std::endl;
	std::cout << tf1 - tf2 << std::endl;
}

TEST(TestAddDisplacement, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	StateRepresentation::CartesianTwist vel("t1");
	vel.set_linear_velocity(Eigen::Vector3d(0.1,0.1,0.1));
	vel.set_angular_velocity(Eigen::Vector3d(0.1,0.1,0));

	std::chrono::milliseconds dt1(10);
	std::cout << tf1 + dt1 * vel << std::endl;

	std::chrono::milliseconds dt2(1000);
	std::cout << tf1 + dt2 * vel << std::endl;

	std::chrono::seconds dt3(1);
	std::cout << tf1 + dt3 * vel << std::endl;
}

TEST(TestPoseToVelocity, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	Eigen::Vector3d pos2(1,0,0);
	Eigen::Quaterniond rot2(0,1,0,0); 
	StateRepresentation::CartesianPose tf2("t1", pos2, rot2);

	std::chrono::seconds dt1(1);
	std::cout << (tf1 - tf2) / dt1 << std::endl;

	std::chrono::seconds dt2(10);
	std::cout << (tf1 - tf2) / dt2 << std::endl;

	std::chrono::milliseconds dt3(100);
	std::cout << (tf1 - tf2) / dt3 << std::endl;
}

TEST(TestImplicitConversion, PositiveNos)
{
	Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();
	Eigen::Quaterniond rot1 = Eigen::Quaterniond::Identity(); 
	StateRepresentation::CartesianPose tf1("t1", pos1, rot1);

	StateRepresentation::CartesianTwist vel("t1");
	vel.set_linear_velocity(Eigen::Vector3d(0.1,0.1,0.1));
	vel.set_angular_velocity(Eigen::Vector3d(0.1,0.1,0));

	tf1 += vel;

	std::cout << tf1 << std::endl;
}

TEST(TestVelocityClamping, PositiveNos)
{
	StateRepresentation::CartesianTwist vel("test", Eigen::Vector3d(1,-2,3), Eigen::Vector3d(1,2,-3));
	vel.clamp(1, 0.5);

	std::cout << vel << std::endl;
	EXPECT_TRUE(vel.get_linear_velocity().norm() <= 1);
	EXPECT_TRUE(vel.get_angular_velocity().norm() <= 0.5);

	vel *= 0.01;

	std::cout << vel.clamped(1, 0.5, 0.1, 0.1) << std::endl;
	for(int i=0; i<3; ++i)
	{
		EXPECT_TRUE(vel.clamped(1, 0.5, 0.1, 0.1).get_linear_velocity()(i) == 0);
		EXPECT_TRUE(vel.clamped(1, 0.5, 0.1, 0.1).get_angular_velocity()(i) == 0);
	}
}

TEST(TestPoseDistance, PositiveNos)
{
	StateRepresentation::CartesianPose p1("test", Eigen::Vector3d::Zero());
	StateRepresentation::CartesianPose p2("test", Eigen::Vector3d(1,0,0));
	StateRepresentation::CartesianPose p3("test", Eigen::Vector3d(1,0,0), Eigen::Quaterniond(0,1,0,0));

	Eigen::Array2d d1 = dist(p1, p2);
	Eigen::Array2d d2 = p1.dist(p2);

	EXPECT_TRUE(d1.isApprox(d2));
	EXPECT_TRUE(d1(0) == 1);
	EXPECT_TRUE(d1(1) == 0);

	Eigen::Array2d d3 = dist(p1, p3);
	EXPECT_TRUE(abs(d3(1) - 3.14159) < 1e10-3);	
}

TEST(TestTwistOperatorsWithEigen, PositiveNos)
{
	Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
	StateRepresentation::CartesianTwist twist("test");
	twist = vec;
	EXPECT_TRUE((vec-twist.get_twist()).norm() < 1e-4);
	twist = vec.array();
	EXPECT_TRUE((vec-twist.get_twist()).norm() < 1e-4);

	twist += vec;
	EXPECT_TRUE((2 * vec-twist.get_twist()).norm() < 1e-4);

	Eigen::Matrix<double, 6, 1> arr = Eigen::Matrix<double, 6, 1>::Random();
	twist = twist + arr;

	EXPECT_TRUE(((2 * vec + arr)-twist.get_twist()).norm() < 1e-4);

	twist = arr + twist;
	EXPECT_TRUE(((2 * vec + 2 * arr)-twist.get_twist()).norm() < 1e-4);

	twist -= arr;
	EXPECT_TRUE(((2 * vec + arr)-twist.get_twist()).norm() < 1e-4);

	twist = twist - vec;
	EXPECT_TRUE(((vec + arr)-twist.get_twist()).norm() < 1e-4);

	twist = vec - twist;
	EXPECT_TRUE((arr+twist.get_twist()).norm() < 1e-4);
}

TEST(TestWrenchOperatorsWithEigen, PositiveNos)
{
	Eigen::Matrix<double, 6, 1> vec = Eigen::Matrix<double, 6, 1>::Random();
	StateRepresentation::CartesianWrench wrench("test");
	wrench = vec;
	EXPECT_TRUE((vec-wrench.get_wrench()).norm() < 1e-4);
	wrench = vec.array();
	EXPECT_TRUE((vec-wrench.get_wrench()).norm() < 1e-4);

	wrench += vec;
	EXPECT_TRUE((2 * vec-wrench.get_wrench()).norm() < 1e-4);

	Eigen::Matrix<double, 6, 1> arr = Eigen::Matrix<double, 6, 1>::Random();
	wrench = wrench + arr;

	EXPECT_TRUE(((2 * vec + arr)-wrench.get_wrench()).norm() < 1e-4);

	wrench = arr + wrench;
	EXPECT_TRUE(((2 * vec + 2 * arr)-wrench.get_wrench()).norm() < 1e-4);

	wrench -= arr;
	EXPECT_TRUE(((2 * vec + arr)-wrench.get_wrench()).norm() < 1e-4);

	wrench = wrench - vec;
	EXPECT_TRUE(((vec + arr)-wrench.get_wrench()).norm() < 1e-4);

	wrench = vec - wrench;
	EXPECT_TRUE((arr+wrench.get_wrench()).norm() < 1e-4);
}

TEST(TestFilter, PositiveNos)
{
	StateRepresentation::CartesianPose tf1("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());
	StateRepresentation::CartesianPose tf2("t1", Eigen::Vector3d::Random(), Eigen::Quaterniond::UnitRandom());

	for (int i = 0; i <1000; ++i)
	{
		StateRepresentation::CartesianPose temp = tf1;

		double alpha = 0.1;
		tf1 = (1-alpha) * tf1 + alpha * tf2;
		EXPECT_TRUE((tf1.get_position() - ((1-alpha) * temp.get_position() + alpha * tf2.get_position())).norm() < 1e-4);
	}

	EXPECT_TRUE(dist(tf1, tf2)(0) < 1e-4);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}