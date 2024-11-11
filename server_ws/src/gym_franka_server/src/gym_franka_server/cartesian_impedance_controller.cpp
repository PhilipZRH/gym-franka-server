// Adapted from Franka ROS example controller.

#include <gym_franka_server/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <franka_example_controllers/pseudo_inversion.h>

namespace gym_franka_server {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                        ros::NodeHandle& node_handle) {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    sub_equilibrium_pose_ = node_handle.subscribe(
        "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
            "aborting controller init!");
        return false;
    }

    // Fetch joint limits
    for (size_t i = 0; i < 7; i++) {
        if (!node_handle.getParam(std::string("/joint/limit/lower").insert(6, std::to_string(i + 1)), q_min_[i]) ||
            !node_handle.getParam(std::string("/joint/limit/upper").insert(6, std::to_string(i + 1)), q_max_[i])) {
            ROS_ERROR("CartesianImpedanceController: Failed to read joint limits.");
            return false;
        }
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "CartesianImpedanceController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }

    // Set collision threasholds.
    ros::ServiceClient client = node_handle.serviceClient<franka_msgs::SetForceTorqueCollisionBehavior>(
        "/franka_control/set_force_torque_collision_behavior");
    franka_msgs::SetForceTorqueCollisionBehavior srv;
    
    // 2x default
    // srv.request.upper_force_thresholds_nominal = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};
    // srv.request.lower_force_thresholds_nominal = {20.0, 20.0, 20.0, 25.0, 25.0, 25.0};
    // srv.request.upper_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
    // srv.request.lower_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
    srv.request.upper_force_thresholds_nominal = {40.0, 40.0, 40.0, 25.0, 25.0, 25.0};
    srv.request.lower_force_thresholds_nominal = {40.0, 40.0, 40.0, 25.0, 25.0, 25.0};
    srv.request.upper_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
    srv.request.lower_torque_thresholds_nominal = {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};

    if (client.call(srv)) {
        ROS_INFO("[Collision Setting]: %i", srv.response.success);
    } else {
        ROS_ERROR("[Cartesian Impedance Controller] Failed to set collision behavior.");
        return false;
    }

    // dynamic_reconfigure_compliance_param_node_ =
    //     ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

    // dynamic_server_compliance_param_ = std::make_unique<
    //     dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

    //     dynamic_reconfigure_compliance_param_node_);
    // dynamic_server_compliance_param_->setCallback(
    //     boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << 200 * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << 20 * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(200) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(20) * Eigen::Matrix3d::Identity();
    nullspace_stiffness_target_ = 0.5;

    return true;
}

void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
}

void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                          const ros::Duration& /*period*/) {
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.rotation() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7), tau_joint_limit(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // Cartesian PD control with damping ratio = 1
    // Adding force limit
    Eigen::Matrix<double, 6, 1> cartesian_force_torque, cft_cap_low, cft_cap_high;
    // cft_cap_low << -20, -10, -10, -5, -5, -5;
    // cft_cap_high << 20, 10, 10, 5, 5, 5;
    cft_cap_low << -30, -15, -15, -7.5, -7.5, -7.5;
    cft_cap_high << 30, 15, 15, 7.5, 7.5, 7.5;
    cartesian_force_torque << -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq);
    for (size_t i = 0; i < 6; i++) {
        if (cartesian_force_torque[i] < cft_cap_low[i]) {
            cartesian_force_torque[i] = cft_cap_low[i];
            // std::cout << "force limit breached" << std::endl;
        }
        if (cartesian_force_torque[i] > cft_cap_high[i]) {
            cartesian_force_torque[i] = cft_cap_high[i];
            // std::cout << "force limit breached" << std::endl;
        }
    }
    tau_task << jacobian.transpose() * cartesian_force_torque;

    // nullspace PD control with damping ratio = 1
    Eigen::VectorXd q_null_dist(7);
    q_null_dist.setZero();
    q_null_dist[0] = (q_d_nullspace_ - q)[0];
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                     (nullspace_stiffness_ * q_null_dist - (2.0 * sqrt(nullspace_stiffness_)) * dq);

    // Add counter torque when joint limit slack is breached.
    tau_joint_limit.setZero();
    double q_dist_to_min, q_dist_to_max;
    for (size_t i = 0; i < 7; i++) {
        q_dist_to_min = q[i] - q_min_[i];
        q_dist_to_max = q_max_[i] - q[i];
        if (q_dist_to_min < q_limit_slack_) {
            tau_joint_limit[i] = (q_limit_slack_ - q_dist_to_min) * q_limit_avoidance_stiffness_;
        } else if (q_dist_to_max < q_limit_slack_) {
            tau_joint_limit[i] = -(q_limit_slack_ - q_dist_to_max) * q_limit_avoidance_stiffness_;
        }
    }

    // Desired torque
    tau_d << tau_task + tau_nullspace + coriolis + tau_joint_limit;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d(i));
    }

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    // Safety region
    position_d_target_[0] = std::min(0.8, std::max(0.3, position_d_target_[0]));
    position_d_target_[1] = std::min(0.5, std::max(-0.5, position_d_target_[1]));
    position_d_target_[2] = std::min(0.5, std::max(0., position_d_target_[2]));
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}

// void CartesianImpedanceController::complianceParamCallback(
//     franka_example_controllers::compliance_paramConfig& config,
//     uint32_t /*level*/) {
//     cartesian_stiffness_target_.setIdentity();
//     cartesian_stiffness_target_.topLeftCorner(3, 3)
//         << config.translational_stiffness * Eigen::Matrix3d::Identity();
//     cartesian_stiffness_target_.bottomRightCorner(3, 3)
//         << config.rotational_stiffness * Eigen::Matrix3d::Identity();
//     cartesian_damping_target_.setIdentity();
//     // Damping ratio = 1
//     cartesian_damping_target_.topLeftCorner(3, 3)
//         << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
//     cartesian_damping_target_.bottomRightCorner(3, 3)
//         << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
//     nullspace_stiffness_target_ = config.nullspace_stiffness;
// }

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        position_and_orientation_d_target_mutex_);
    position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }
}

}  // namespace gym_franka_server

PLUGINLIB_EXPORT_CLASS(gym_franka_server::CartesianImpedanceController,
                       controller_interface::ControllerBase)
