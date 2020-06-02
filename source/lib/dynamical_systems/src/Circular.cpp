#include "dynamical_systems/Circular.hpp"

namespace DynamicalSystems
{
	Circular::Circular(const StateRepresentation::CartesianState& center, double radius, double gain, double circular_velocity):
	limit_cycle_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::Ellipsoid>>("limit_cycle", StateRepresentation::Ellipsoid(center.get_name(), center.get_reference_frame()))),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain)),
	circular_velocity_(std::make_shared<StateRepresentation::Parameter<double>>("circular_velocity", circular_velocity))
	{
		this->limit_cycle_->get_value().set_center_state(center);
		this->limit_cycle_->get_value().set_axis_lengths({radius, radius});
	}

	Circular::Circular(const StateRepresentation::Ellipsoid& limit_cycle, double gain, double circular_velocity):
	limit_cycle_(std::make_shared<StateRepresentation::Parameter<StateRepresentation::Ellipsoid>>(" limit_cycle", limit_cycle)),
	gain_(std::make_shared<StateRepresentation::Parameter<double>>("gain", gain)),
	circular_velocity_(std::make_shared<StateRepresentation::Parameter<double>>("circular_velocity", circular_velocity))
	{}

	const StateRepresentation::CartesianState Circular::evaluate(const StateRepresentation::CartesianState& state) const
	{
		// put the point in the reference of the center
		StateRepresentation::CartesianPose pose = static_cast<const StateRepresentation::CartesianPose&>(state);
		pose = this->get_center().inverse() * pose;

		StateRepresentation::CartesianTwist velocity(pose.get_name(), pose.get_reference_frame());
		Eigen::Vector3d linear_velocity;
		linear_velocity(2) = -this->get_gain() * pose.get_position()(2);

		float radius = sqrt(pose.get_position()(0) * pose.get_position()(0) + pose.get_position()(1) * pose.get_position()(1));
		float theta = atan2(pose.get_position()(1), pose.get_position()(0));
		float angular_gain = this->get_circular_velocity();

		std::vector<double> radiuses = this->get_radiuses();
		double radius_desired = (radiuses[0] * radiuses[1]) / sqrt(radiuses[0]*radiuses[0]*sin(theta)*sin(theta) + radiuses[1]*radiuses[1]*cos(theta)*cos(theta));

		double dradius = -this->get_gain() * (radius - radius_desired);
		double dtheta = -angular_gain * exp(-dradius * dradius) / radius_desired;

		linear_velocity(0) = dradius * cos(theta) - radius * dtheta * sin(theta);
		linear_velocity(1) = dradius * sin(theta) + radius * dtheta * cos(theta);

		velocity.set_linear_velocity(linear_velocity);
		velocity.set_angular_velocity(Eigen::Vector3d::Zero());

		//compute back the linear velocity in the desired frame
		velocity = this->get_center() * velocity;
		return velocity;
	}

	const std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> Circular::get_parameters() const
	{
		std::list<std::shared_ptr<StateRepresentation::ParameterInterface>> param_list;
		param_list.push_back(this->limit_cycle_);
		param_list.push_back(this->gain_);
		param_list.push_back(this->circular_velocity_);
		return param_list;
	}
}