// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers/cartesian_impedance_controller_cino.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "utils/pseudo_inversion.h"

namespace panda_controllers {


bool CartesianImpedanceControllerCino::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;
  
  // Name space extraction for add a prefix to the topic name
  int n = 0;
  std::string name_space;
  name_space = node_handle.getNamespace();
  n = name_space.find("/", 2);
  name_space = name_space.substr(0,n);

  sub_equilibrium_pose_ = node_handle.subscribe(
      name_space+"/equilibrium_pose", 1, &CartesianImpedanceControllerCino::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_desired_stiffness_ = node_handle.subscribe(
      name_space+"/desired_stiffness", 1, &CartesianImpedanceControllerCino::desiredStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_stiffness_scale_factor_ = node_handle.subscribe(
      name_space+"/stiffness_scale_factor", 1, &CartesianImpedanceControllerCino::stiffnessScaleFactorCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/franka_ee_pose", 1);

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceControllerCino: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceControllerCino: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerCino: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerCino: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerCino: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerCino: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerCino: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceControllerCino: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_.setIdentity();
  cartesian_damping_target_.setIdentity();

  
  cartesian_stiffness_.topLeftCorner(3, 3) << kDefaultStiffness*Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << (kDefaultStiffness/stiffness_scale_factor_)*Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_ = cartesian_stiffness_;
  // Damping ratio = 1

  cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt(kDefaultStiffness)*Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(kDefaultStiffness/stiffness_scale_factor_)*Eigen::Matrix3d::Identity();
  cartesian_damping_target_ = cartesian_damping_;

  return true;
}

void CartesianImpedanceControllerCino::starting(const ros::Time& /*time*/) {
  
  ROS_INFO("Cartesian impedance controller Cino starting...");
  
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;

}

void CartesianImpedanceControllerCino::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());


  geometry_msgs::PoseStamped msg_endeffector_pose_;

  msg_endeffector_pose_.pose.position.x = position(0);
  msg_endeffector_pose_.pose.position.y = position(1);
  msg_endeffector_pose_.pose.position.z = position(2);
  msg_endeffector_pose_.pose.orientation.x = orientation.x();
  msg_endeffector_pose_.pose.orientation.y = orientation.y();
  msg_endeffector_pose_.pose.orientation.z = orientation.z();
  msg_endeffector_pose_.pose.orientation.w = orientation.w();

  pub_endeffector_pose_.publish(msg_endeffector_pose_);


  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  Eigen::Matrix<double, 6, 1> error_prev;
  Eigen::Matrix<double, 6, 1> dot_error;
  Eigen::Matrix<double, 6, 1> dot_error_filt;

  error.head(3) << position - position_d_;

  
  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }


  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);


  tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  //tau_d << tau_task + coriolis;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceControllerCino::saturateTorqueRate(
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

void CartesianImpedanceControllerCino::desiredStiffnessCallback(
    const geometry_msgs::Vector3StampedConstPtr& msg) {

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0,0) = msg->vector.x;
  cartesian_stiffness_target_(1,1) = msg->vector.y;
  cartesian_stiffness_target_(2,2) = msg->vector.z;
  
  cartesian_stiffness_target_.bottomRightCorner(3, 3) << cartesian_stiffness_target_.topLeftCorner(3, 3)/stiffness_scale_factor_;
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1

  cartesian_damping_target_(0,0) = 2.0 * sqrt(msg->vector.x);
  cartesian_damping_target_(1,1) = 2.0 * sqrt(msg->vector.y);
  cartesian_damping_target_(2,2) = 2.0 * sqrt(msg->vector.z);
  
  cartesian_damping_target_(3,3) = 2.0 * sqrt(msg->vector.x/stiffness_scale_factor_);
  cartesian_damping_target_(4,4) = 2.0 * sqrt(msg->vector.y/stiffness_scale_factor_);
  cartesian_damping_target_(5,5) = 2.0 * sqrt(msg->vector.z/stiffness_scale_factor_);
}

void CartesianImpedanceControllerCino::stiffnessScaleFactorCallback(
  const std_msgs::Float64::ConstPtr& msg) {
  
  stiffness_scale_factor_ = static_cast<double>(msg->data);

  cartesian_stiffness_target_.bottomRightCorner(3, 3) << cartesian_stiffness_target_.topLeftCorner(3, 3)/stiffness_scale_factor_;

  cartesian_damping_target_(3,3) = 2.0 * sqrt(cartesian_stiffness_target_(0,0)/stiffness_scale_factor_);
  cartesian_damping_target_(4,4) = 2.0 * sqrt(cartesian_stiffness_target_(1,1)/stiffness_scale_factor_);
  cartesian_damping_target_(5,5) = 2.0 * sqrt(cartesian_stiffness_target_(2,2)/stiffness_scale_factor_);
}

void CartesianImpedanceControllerCino::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }

  Eigen::AngleAxisd aa_orientation_recieved(orientation_d_target_);
}

}  // namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::CartesianImpedanceControllerCino,
                       controller_interface::ControllerBase)
