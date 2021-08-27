// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_controllers/cartesian_pose_controller_cino.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace panda_controllers {

bool CartesianPoseControllerCino::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPoseControllerCino: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseControllerCino: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPoseControllerCino: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  // Name space extraction for add a prefix to the topic name
  int n = 0;
  std::string name_space;
  name_space = node_handle.getNamespace();
  n = name_space.find("/", 2);
  name_space = name_space.substr(0,n);

  sub_equilibrium_pose_ = node_handle.subscribe(
      name_space+"/equilibrium_pose", 1, &CartesianPoseControllerCino::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/franka_ee_pose", 1);

  return true;
}

void CartesianPoseControllerCino::starting(const ros::Time& /* time */) {
  // get roboto initial pose
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  // set equilibrium point to current state
  position_d_        << initial_pose_[12], initial_pose_[13], initial_pose_[14];
  position_d_target_ = position_d_;
}

void CartesianPoseControllerCino::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  // get robot pose
  std::array<double, 16> pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  // convert to Eigen
  Eigen::Vector3d position;
  position << pose[12], pose[13], pose[14];

  geometry_msgs::PoseStamped msg_endeffector_pose;

  msg_endeffector_pose.pose.position.x = position(0);
  msg_endeffector_pose.pose.position.y = position(1);
  msg_endeffector_pose.pose.position.z = position(2);

  pub_endeffector_pose_.publish(msg_endeffector_pose);
  
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  
  std::array<double, 16> new_pose{pose};
  new_pose[12] = position_d_.x();
  new_pose[13] = position_d_.y();
  new_pose[14] = position_d_.z();
  
  cartesian_pose_handle_->setCommand(new_pose);
}

void CartesianPoseControllerCino::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

  // Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  // orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
  //     msg->pose.orientation.z, msg->pose.orientation.w;

  // if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
  //   orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  // }
}

}  // namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::CartesianPoseControllerCino,
                       controller_interface::ControllerBase)
