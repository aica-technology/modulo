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
		this->set_zero();
	}

	void CartesianState::set_zero()
	{
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

	CartesianState& CartesianState::operator*=(const CartesianState& state)
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		if(this->get_name() != state.get_reference_frame()) throw IncompatibleReferenceFramesException("Expected " + this->get_name() + ", got " + state.get_reference_frame());
		this->set_name(state.get_name());
		// intermediate variables for f_S_b
		Eigen::Vector3d f_P_b = this->get_position();
		Eigen::Quaterniond f_R_b = this->get_orientation();
		Eigen::Vector3d f_v_b = this->get_linear_velocity();
		Eigen::Vector3d f_omega_b = this->get_angular_velocity();
		Eigen::Vector3d f_a_b = this->get_linear_acceleration();
		Eigen::Vector3d f_alpha_b = this->get_angular_acceleration();
		// intermediate variables for b_S_c
		Eigen::Vector3d b_P_c = state.get_position();
		Eigen::Quaterniond b_R_c = (this->get_orientation().dot(state.get_orientation()) > 0) ? state.get_orientation() : Eigen::Quaterniond(-state.get_orientation().coeffs());
		Eigen::Vector3d b_v_c = state.get_linear_velocity();
		Eigen::Vector3d b_omega_c = state.get_angular_velocity();
		Eigen::Vector3d b_a_c = state.get_linear_acceleration();
		Eigen::Vector3d b_alpha_c = state.get_angular_acceleration();
		// pose
		this->set_position(f_P_b + f_R_b * b_P_c);
		this->set_orientation(f_R_b * b_R_c);
		// twist
		this->set_linear_velocity(f_v_b + f_R_b * b_v_c + f_omega_b.cross(f_R_b * b_P_c));
		this->set_angular_velocity(f_omega_b + f_R_b * b_omega_c);
		// acceleration
		this->set_linear_acceleration(f_a_b + f_R_b * b_a_c + f_alpha_b.cross(f_R_b * b_P_c) + 2 * f_omega_b.cross(f_R_b * b_v_c) + f_omega_b.cross(f_omega_b.cross(f_R_b * b_P_c)));
		this->set_angular_acceleration(f_alpha_b + f_R_b * b_alpha_c + f_omega_b.cross(f_R_b * b_omega_c));
		// wrench
		//TODO
		return (*this);
	}

	const CartesianState CartesianState::operator*(const CartesianState& state) const
	{
		CartesianState result(*this);
		result *= state;
		return result;
	}

	const CartesianState CartesianState::inverse() const
	{
		CartesianState result(*this);
		// inverse name and reference frame
		std::string ref = result.get_reference_frame();
		std::string name = result.get_name();
		result.set_reference_frame(name);
		result.set_name(ref);
		// intermediate variables for f_S_b
		Eigen::Vector3d f_P_b = this->get_position();
		Eigen::Quaterniond f_R_b = this->get_orientation();
		Eigen::Vector3d f_v_b = this->get_linear_velocity();
		Eigen::Vector3d f_omega_b = this->get_angular_velocity();
		Eigen::Vector3d f_a_b = this->get_linear_acceleration();
		Eigen::Vector3d f_alpha_b = this->get_angular_acceleration();
		// computation for b_S_f
		Eigen::Quaterniond b_R_f = f_R_b.conjugate();
		Eigen::Vector3d b_P_f = b_R_f * (-f_P_b);
		Eigen::Vector3d b_v_f = b_R_f * (-f_v_b);
		Eigen::Vector3d b_omega_f = b_R_f * (-f_omega_b);
		Eigen::Vector3d b_a_f = b_R_f * f_a_b; // not sure if minus is needed
		Eigen::Vector3d b_alpha_f = b_R_f * f_alpha_b; // no minus for sure
		// wrench
		//TODO
		// collect the results
		result.set_position(b_P_f);
		result.set_orientation(b_R_f);
		result.set_linear_velocity(b_v_f);
		result.set_angular_velocity(b_omega_f);
		result.set_linear_acceleration(b_a_f);
		result.set_angular_acceleration(b_alpha_f);
		return result;
	}

	double CartesianState::dist(const CartesianState& state, const std::string& distance_type) const
	{
		// sanity check
		if(this->is_empty()) throw EmptyStateException(this->get_name() + " state is empty");
		if(state.is_empty()) throw EmptyStateException(state.get_name() + " state is empty");
		if(!(this->get_reference_frame() == state.get_reference_frame())) throw IncompatibleReferenceFramesException("The two states do not have the same reference frame");
		// calculation
		double result = 0;
		if (distance_type == "position" || distance_type == "pose" || distance_type == "all")
		{
			result += (this->get_position() - state.get_position()).norm();
		}
		if (distance_type == "orientation" || distance_type == "pose" || distance_type == "all")
		{
			// https://math.stackexchange.com/questions/90081/quaternion-distance for orientation
			double inner_product = this->get_orientation().dot(state.get_orientation());
			result += acos(2 * inner_product * inner_product - 1);
		}
		if (distance_type == "linear_velocity" || distance_type == "twist" || distance_type == "all")
		{
			result += (this->get_linear_velocity() - state.get_linear_velocity()).norm();
		}
		if (distance_type == "angular_velocity" || distance_type == "twist" || distance_type == "all")
		{
			result += (this->get_angular_velocity() - state.get_angular_velocity()).norm();
		}
		if (distance_type == "linear_acceleration" || distance_type == "acceleration" || distance_type == "all")
		{
			result += (this->get_linear_acceleration() - state.get_linear_acceleration()).norm();
		}
		if (distance_type == "angular_acceleration" || distance_type == "acceleration" || distance_type == "all")
		{
			result += (this->get_angular_acceleration() - state.get_angular_acceleration()).norm();
		}
		if (distance_type == "force" || distance_type == "wrench" || distance_type == "all")
		{
			result += (this->get_force() - state.get_force()).norm();
		}
		if (distance_type == "torque" || distance_type == "wrench" || distance_type == "all")
		{
			result += (this->get_torque() - state.get_torque()).norm();
		}
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

	double dist(const CartesianState& s1, const CartesianState& s2, const std::string& distance_type)
	{
		return s1.dist(s2, distance_type);
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
