#include "state_representation/Space/Cartesian/CartesianState.hpp"
#include "state_representation/Exceptions/EmptyStateException.hpp"
#include "state_representation/Exceptions/IncompatibleReferenceFramesException.hpp"
#include "state_representation/Exceptions/NotImplementedException.hpp"

using namespace StateRepresentation::Exceptions;

namespace StateRepresentation 
{
	CartesianState::CartesianState():
	SpatialState(StateType::CARTESIANSTATE)
	{
		this->initialize();
	}

	CartesianState::CartesianState(const std::string& robot_name, const std::string& reference):
	SpatialState(StateType::CARTESIANSTATE, robot_name, reference)
	{
		this->initialize();
	}

	CartesianState::CartesianState(const CartesianState& state):
	SpatialState(state),
	position(state.position), orientation(state.orientation),
	linear_velocity(state.linear_velocity), angular_velocity(state.angular_velocity),
	linear_acceleration(state.linear_acceleration), angular_acceleration(state.angular_acceleration),
	force(state.force), torque(state.torque)
	{}

	void CartesianState::initialize()
	{
		this->State::initialize();
		this->position.setZero();
		this->orientation.setIdentity();
		this->linear_velocity.setZero();
		this->angular_velocity.setZero();
		this->linear_acceleration.setZero();
		this->angular_acceleration.setZero();
		this->force.setZero();
		this->torque.setZero();
	}

	CartesianState& CartesianState::operator*=(double lambda)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		// operation
		this->set_position(lambda * this->get_position());
		// calculate the scaled rotation as a displacement from identity
		Eigen::Quaterniond w = MathTools::log(this->get_orientation());
		// calculate the orientation corresponding to the scaled velocity
		this->set_orientation(MathTools::exp(w, lambda / 2.));
		// calculate the other vectors normally
		this->set_linear_velocity(lambda * this->get_linear_velocity());
		this->set_angular_velocity(lambda * this->get_angular_velocity());
		this->set_linear_acceleration(lambda * this->get_linear_acceleration());
		this->set_angular_acceleration(lambda * this->get_angular_acceleration());
		this->set_force(lambda * this->get_force());
		this->set_torque(lambda * this->get_torque());
		return (*this);
	}

	const CartesianState CartesianState::operator*(double lambda) const
	{
		CartesianState result(*this);
		result *= lambda;
		return result;
	}

	const CartesianState CartesianState::copy() const
	{
		CartesianState result(*this);
		return result;
	}

	double CartesianState::dist(const CartesianState& state) const
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		if(!(this->get_reference_frame() == state.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// calculation
		double result = 0;
		// euclidean distance for position
		result += (this->get_position() - state.get_position()).norm();
		// https://math.stackexchange.com/questions/90081/quaternion-distance for orientation
		double inner_product = this->get_orientation().dot(state.get_orientation());
		result += acos(2 * inner_product * inner_product - 1);
		// distance for velocity
		result += (this->get_linear_velocity() - state.get_linear_velocity()).norm();
		result += (this->get_angular_velocity() - state.get_angular_velocity()).norm();
		// distance for acceleration
		result += (this->get_linear_acceleration() - state.get_linear_acceleration()).norm();
		result += (this->get_angular_acceleration() - state.get_angular_acceleration()).norm();
		// distance for force
		result += (this->get_force() - state.get_force()).norm();
		result += (this->get_torque() - state.get_torque()).norm();
		return result;
	}

	std::ostream& operator<<(std::ostream& os, const CartesianState& state) 
	{ 
		if(state.is_empty())
		{
			os << "Empty CartesianState";
		}
		else
		{
			os << state.get_name() << " CartesianState expressed in " << state.get_reference_frame() << " frame" << std::endl;
	  		os << "position: (" << state.position(0) << ", ";
	  		os << state.position(1) << ", ";
	  		os << state.position(2) << ")" << std::endl;
	  		os << "orientation: (" <<state.orientation.w() << ", ";
	  		os << state.orientation.x() << ", ";
	  		os << state.orientation.y() << ", ";
	  		os << state.orientation.z() << ")";
	  		Eigen::AngleAxisd axis_angle(state.orientation);
	  		os << " <=> theta: " << axis_angle.angle() << ", ";
	  		os << "axis: (" << axis_angle.axis()(0) << ", ";
	  		os << axis_angle.axis()(1) << ", ";
	  		os << axis_angle.axis()(2) << ")" << std::endl;
	  		os << "linear velocity: (" << state.linear_velocity(0) << ", ";
	  		os << state.linear_velocity(1) << ", ";
	  		os << state.linear_velocity(2) << ")" << std::endl;
	  		os << "angular velocity: (" << state.angular_velocity(0) << ", ";
	  		os << state.angular_velocity(1) << ", ";
	  		os << state.angular_velocity(2) << ")" << std::endl;
	  		os << "linear acceleration: (" << state.linear_acceleration(0) << ", ";
	  		os << state.linear_acceleration(1) << ", ";
	  		os << state.linear_acceleration(2) << ")" << std::endl;
	  		os << "angular acceleration: (" << state.angular_acceleration(0) << ", ";
	  		os << state.angular_acceleration(1) << ", ";
	  		os << state.angular_acceleration(2) << ")" << std::endl;
	  		os << "force: (" << state.force(0) << ", ";
	  		os << state.force(1) << ", ";
	  		os << state.force(2) << ")" << std::endl;
	  		os << "torque: (" << state.torque(0) << ", ";
	  		os << state.torque(1) << ", ";
	  		os << state.torque(2) << ")";
	  	}
  		return os;
	}

	const CartesianState operator*(double lambda, const CartesianState& state)
	{
		return state * lambda;
	}

	double dist(const CartesianState& s1, const CartesianState& s2)
	{
		return s1.dist(s2);
	}

	const std::vector<double> CartesianState::to_std_vector() const
	{
		throw(NotImplementedException("to_std_vector() is not implemented for the base CartesianState class"));
		return std::vector<double>();
	}

	void CartesianState::from_std_vector(const std::vector<double>&)
	{
		throw(NotImplementedException("from_std_vector() is not implemented for the base CartesianState class"));
	}
}
