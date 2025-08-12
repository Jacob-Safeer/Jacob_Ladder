#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <limits>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TrackFollow : public px4_ros2::ModeBase
{
public:
	explicit TrackFollow(rclcpp::Node& node);

	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// See ModeBase
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	struct ArucoTag {
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };
	};

	void loadParameters();
	ArucoTag getTagWorld(const ArucoTag& tag);

	Eigen::Vector2f calculateVelocitySetpointXY();
	bool checkTargetTimeout();
	bool positionReached(const Eigen::Vector3f& target) const;
	bool xyDistanceTo(const Eigen::Vector3f& target, float thresh) const;

	enum class State {
		Idle,
		Search,     // Fly upward until target is in camera frame
		Approach,   // Position over landing target while maintaining altitude
		Descend,    // Stay over landing target while descending
		Finished
	};

	void switchToState(State state);
	std::string stateName(State state);

	// ROS 2
	rclcpp::Node& _node;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;

	// PX4 ROS 2
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// State machine
	State _state = State::Search;
	bool _search_started = false;

	ArucoTag _tag;
	float _approach_altitude = {};
	bool _land_detected = false;
	bool _target_lost_prev = true;

	// Parameters (common)
	float _param_descent_vel = {};
	float _param_vel_p_gain = {};
	float _param_vel_i_gain = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};

	// --- New Search parameters ---
	float _param_search_climb_speed = 0.7f;  // m/s climb rate
	float _param_search_min_z = -15.0f;      // NED ceiling (more negative = higher)

	// --- Approach tuning (for moving targets) ---
	float _param_approach_p_gain   = 1.6f;
	float _param_approach_max_vel  = 4.0f;
	float _param_approach_lead_time= 0.5f;
	float _param_approach_lead_max = 1.5f;
	float _param_approach_ff_scale = 1.0f;
	bool  _param_approach_use_vel  = true;

	// Moving-target handoff
	bool  _param_use_relative_speed_gate = true;
	float _param_descend_rel_speed_thresh = 0.6f; // m/s

	// PID integrators for velocity hold
	float _vel_x_integral {};
	float _vel_y_integral {};

	// Tag velocity estimation (world/NED frame)
	Eigen::Vector3d _tag_prev_pos = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	rclcpp::Time    _tag_prev_stamp{};
	Eigen::Vector2f _tag_xy_vel {0.f, 0.f};
};

