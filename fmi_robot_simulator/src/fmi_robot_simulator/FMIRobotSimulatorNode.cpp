// Copyright (c) 2019 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fmi_robot_simulator/FMIRobotSimulatorNode.hpp"

#include <cassert>

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "fmi_robot_simulator/FMIAdapter.hpp"

namespace fmi_robot_simulator {

    FMIRobotSimulatorNode::FMIRobotSimulatorNode(const rclcpp::NodeOptions &options)
            : LifecycleNode("fmi_adapter_node", options) {
        // Get parameters from parameter server
        get_node_parameters_interface()->declare_parameter(
                "fmu_path", rclcpp::ParameterValue(""), rcl_interfaces::msg::ParameterDescriptor());
        get_node_parameters_interface()->declare_parameter(
                "step_size", rclcpp::ParameterValue(0.0), rcl_interfaces::msg::ParameterDescriptor());
        get_node_parameters_interface()->declare_parameter(
                "update_period", rclcpp::ParameterValue(0.01), rcl_interfaces::msg::ParameterDescriptor());

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode created!\n");
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    FMIRobotSimulatorNode::on_configure(const rclcpp_lifecycle::State &) {
        // FMU path
        std::string fmuPath;
        rclcpp::Parameter parameter;
        if (get_parameter("fmu_path", parameter)) {
            fmuPath = parameter.as_string();
        } else {
            RCLCPP_ERROR(get_logger(), "Parameter 'fmu_path' not specified!");
            throw std::runtime_error("Parameter 'fmu_path' not specified!");
        }

        // Step size
        double stepSizeAsDouble = 0.0;
        if (get_parameter("step_size", parameter)) {
            stepSizeAsDouble = parameter.as_double();
        }
        rclcpp::Duration stepSize = rclcpp::Duration(1, 0) * stepSizeAsDouble;

        // Configuring the adapter
        adapter_ = std::make_shared<fmi_robot_simulator::FMIAdapter>(get_logger(), fmuPath, stepSize);
        for (const std::string &name: adapter_->getParameterNames()) {
            RCLCPP_DEBUG(get_logger(), "FMU has parameter '%s'", name.c_str());
        }
        adapter_->declareROSParameters(get_node_parameters_interface());
        adapter_->initializeFromROSParameters(get_node_parameters_interface());

        // Creating subscribers
        cmd_vel_sub_ =
                this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1,
                                                                     [this](geometry_msgs::msg::Twist::SharedPtr msg) {
                                                                         adapter_->setInputValue("v", now(),
                                                                                                 msg->linear.x);
                                                                         adapter_->setInputValue("w", now(),
                                                                                                 msg->angular.z);
                                                                     });

        // Creating publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SystemDefaultsQoS());

        adapter_->exitInitializationMode(now());

        std::chrono::nanoseconds updatePeriod(10000000);  // Default is 0.01s
        if (get_parameter("update_period", parameter)) {
            updatePeriod = std::chrono::nanoseconds(static_cast<int64_t>(parameter.as_double() * 1000000000.0));
        }

        timer_ = create_wall_timer(
                updatePeriod, [this]() {
                    rclcpp::Time currentTimepoint = now();
                    if (adapter_->getSimulationTime() < currentTimepoint) {
                        adapter_->doStepsUntil(currentTimepoint);
                    } else {
                        RCLCPP_INFO(get_logger(),
                                    "Simulation time %f is greater than timer's time %f. Is your step size to large?",
                                    adapter_->getSimulationTime().seconds(), currentTimepoint.seconds());
                    }

                    // Publish odometry message
                    nav_msgs::msg::Odometry odom_msg;
                    odom_msg.header.stamp = adapter_->getSimulationTime();
                    odom_msg.pose.pose.position.x = adapter_->getOutputValue("x");
                    odom_msg.pose.pose.position.y = adapter_->getOutputValue("y");
                    odom_msg.pose.pose.position.z = adapter_->getOutputValue("z");
                    odom_msg.pose.pose.orientation.x = adapter_->getOutputValue("qx");
                    odom_msg.pose.pose.orientation.y = adapter_->getOutputValue("qy");
                    odom_msg.pose.pose.orientation.z = adapter_->getOutputValue("qz");
                    odom_msg.pose.pose.orientation.w = adapter_->getOutputValue("qw");
                    odom_msg.twist.twist.linear.x =  adapter_->getOutputValue("vx");
                    odom_msg.twist.twist.linear.y =  adapter_->getOutputValue("vy");
                    odom_msg.twist.twist.linear.z =  adapter_->getOutputValue("vz");
                    odom_msg.twist.twist.angular.x =  adapter_->getOutputValue("wx");
                    odom_msg.twist.twist.angular.y =  adapter_->getOutputValue("wy");
                    odom_msg.twist.twist.angular.z =  adapter_->getOutputValue("wz");

                    if (odom_pub_->is_activated()) {
                        odom_pub_->publish(odom_msg);
                    }

                    // Publish imu message
                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp = adapter_->getSimulationTime();
                    imu_msg.orientation.x = adapter_->getOutputValue("qx");
                    imu_msg.orientation.y = adapter_->getOutputValue("qy");
                    imu_msg.orientation.z = adapter_->getOutputValue("qz");
                    imu_msg.orientation.w = adapter_->getOutputValue("qw");
                    imu_msg.angular_velocity.x = adapter_->getOutputValue("wx");
                    imu_msg.angular_velocity.y = adapter_->getOutputValue("wy");
                    imu_msg.angular_velocity.z = adapter_->getOutputValue("wz");
                    imu_msg.linear_acceleration.x = adapter_->getOutputValue("ax");
                    imu_msg.linear_acceleration.y = adapter_->getOutputValue("ay");
                    imu_msg.linear_acceleration.z = adapter_->getOutputValue("az");

                    if (imu_pub_->is_activated()) {
                        imu_pub_->publish(imu_msg);
                    }
                });

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode configured!\n");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    FMIRobotSimulatorNode::on_activate(const rclcpp_lifecycle::State &) {
        odom_pub_->on_activate();
        imu_pub_->on_activate();

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode activated!\n");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    FMIRobotSimulatorNode::on_deactivate(const rclcpp_lifecycle::State &) {
        odom_pub_->on_deactivate();
        imu_pub_->on_deactivate();

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode deactivated!\n");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    FMIRobotSimulatorNode::on_cleanup(const rclcpp_lifecycle::State &) {
        timer_.reset();
        adapter_.reset();

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode cleaned!\n");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    FMIRobotSimulatorNode::on_shutdown(const rclcpp_lifecycle::State &) {
        timer_.reset();
        adapter_.reset();

        RCLCPP_INFO(get_logger(), "FMIRobotSimulatorNode shut down!\n");

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

}  // namespace fmi_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(fmi_robot_simulator::FMIRobotSimulatorNode)
