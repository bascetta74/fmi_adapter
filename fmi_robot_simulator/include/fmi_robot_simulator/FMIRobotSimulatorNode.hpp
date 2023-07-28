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

#ifndef FMI_ADAPTER__FMIROBOTSIMULATORNODE_HPP_
#define FMI_ADAPTER__FMIROBOTSIMULATORNODE_HPP_

#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace fmi_robot_simulator {

    class FMIAdapter;

    class FMIRobotSimulatorNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit FMIRobotSimulatorNode(const rclcpp::NodeOptions &options);

        RCLCPP_DISABLE_COPY(FMIRobotSimulatorNode)

        virtual ~FMIRobotSimulatorNode() = default;

        virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &);

        virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &);

        virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &);

        virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &);

        virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &);

    private:
        std::shared_ptr <fmi_robot_simulator::FMIAdapter> adapter_{};

        rclcpp::TimerBase::SharedPtr timer_{};

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    };

}  // namespace fmi_robot_simulator

#endif  // FMI_ADAPTER__FMIROBOTSIMULATORNODE_HPP_
