#include "TrackFollow.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>

static const std::string kModeName = "TrackFollowCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

TrackFollow::TrackFollow(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{
	setSkipMessageCompatibilityCheck(); // added this line for message translation to run properly

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>(
		"/target_pose",
		rclcpp::QoS(1).best_effort(),
		std::bind(&TrackFollow::targetPoseCallback, this, std::placeholders::_1)
	);

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
		"/fmu/out/vehicle_land_detected",
		rclcpp::QoS(1).best_effort(),
		std::bind(&TrackFollow::vehicleLandDetectedCallback, this, std::placeholders::_1)
	);

	loadParameters();
}

void TrackFollow::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);

	// New parameters for moving target handoff
	_node.declare_parameter<bool>("use_relative_speed_gate", false);
	_node.declare_parameter<float>("descend_rel_speed_thresh", 0.6);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);

	_node.get_parameter("use_relative_speed_gate", _param_use_relative_speed_gate);
	_node.get_parameter("descend_rel_speed_thresh", _param_descend_rel_speed_thresh);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
	RCLCPP_INFO(_node.get_logger(), "use_relative_speed_gate: %s", _param_use_relative_speed_gate ? "true" : "false");
	RCLCPP_INFO(_node.get_logger(), "descend_rel_speed_thresh: %f", _param_descend_rel_speed_thresh);
}

void TrackFollow::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void TrackFollow::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		ArucoTag tag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// Save tag position/orientation in NED world frame
		ArucoTag world_tag = getTagWorld(tag);

		// Estimate tag XY velocity in world frame
		if (_tag_prev_stamp.nanoseconds() > 0) {
			const double dt = (world_tag.timestamp - _tag_prev_stamp).seconds();
			if (dt > 0.0 &&
			    !std::isnan(_tag_prev_pos.x()) &&
			    !std::isnan(world_tag.position.x()))
			{
				_tag_xy_vel.x() = static_cast<float>((world_tag.position.x() - _tag_prev_pos.x()) / dt);
				_tag_xy_vel.y() = static_cast<float>((world_tag.position.y() - _tag_prev_pos.y()) / dt);
			}
		}

		_tag_prev_pos   = world_tag.position;
		_tag_prev_stamp = world_tag.timestamp;

		_tag = world_tag;
	}
}

TrackFollow::ArucoTag TrackFollow::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	     1,  0, 0,
	     0,  0, 1;
	Eigen::Quaterniond quat_NED(R);

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform  = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform    = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position  = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void TrackFollow::onActivate()
{
	generateSearchWaypoints();
	_search_started = true;
	switchToState(State::Search);
}

void TrackFollow::onDeactivate()
{
	// No-op
}

void TrackFollow::updateSetpoint(float dt_s)
{
	bool target_lost = checkTargetTimeout();

	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
	}

	_target_lost_prev = target_lost;

	// State machine
	switch (_state) {
	case State::Idle: {
		// No-op -- just spin
		break;
	}

	case State::Search: {

		if (!std::isnan(_tag.position.x())) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		auto waypoint_position = _search_waypoints[_search_waypoint_index];

		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			// If we have searched all waypoints, start over
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Approach using position setpoints, constantly chasing current tag XY at fixed altitude
		const auto target_position = Eigen::Vector3f(
			static_cast<float>(_tag.position.x()),
			static_cast<float>(_tag.position.y()),
			_approach_altitude
		);

		_trajectory_setpoint->updatePosition(target_position);

		// Compute relative XY speed (vehicle - tag)
		const Eigen::Vector2f veh_vxy{
			_vehicle_local_position->velocityNed().x(),
			_vehicle_local_position->velocityNed().y()
		};
		const float rel_speed = (veh_vxy - _tag_xy_vel).norm();

		// Exit criteria: close laterally AND (optionally) relative speed small
		const bool close_xy = xyDistanceTo(target_position, _param_delta_position);
		const bool speed_ok = (!_param_use_relative_speed_gate) ||
		                      (rel_speed < _param_descend_rel_speed_thresh);

		if (close_xy && speed_ok) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Descend using velocity setpoints and P velocity controller for XY
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(
			Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel),
			std::nullopt,
			px4_ros2::quaternionToYaw(_tag.orientation)
		);

		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f TrackFollow::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// Clamp to velocity limits
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool TrackFollow::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void TrackFollow::generateSearchWaypoints()
{
	// Generate spiral search waypoints in NED
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Calculate number of layers
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ?
	                 1 : (static_cast<int>((min_z - current_z) / layer_spacing) / 2);

	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// push outwards
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// go down a layer for inward spiral
		current_z += layer_spacing;

		// reverse for inward spiral
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// adjust z for inward
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = static_cast<float>(current_z);
		}

		// push inward
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// next outward layer depth
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool TrackFollow::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// NOTE: this does NOT handle a moving target!
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

bool TrackFollow::xyDistanceTo(const Eigen::Vector3f& target, float thresh) const
{
	const auto p = _vehicle_local_position->positionNed();
	const Eigen::Vector2f d{p.x() - target.x(), p.y() - target.y()};
	return d.norm() < thresh;
}

std::string TrackFollow::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}

void TrackFollow::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<TrackFollow>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
