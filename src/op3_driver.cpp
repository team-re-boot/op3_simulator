// Copyright 2024 Team Reboot.
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
// limitations under the License

#include <cnoid/EigenUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Joystick>
#include <cnoid/Link>
#include <cnoid/SimpleController>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cnoid/src/Body/SimpleController.h>

#include <cstdio>

struct Joint
{
  int id;
  std::string name;
  double position;
  double velocity;
  double effort;
};

class OP3Driver : public cnoid::SimpleController
{
public:
  virtual bool configure(cnoid::SimpleControllerConfig * config) override
  {
    node_ = std::make_shared<rclcpp::Node>(config->controllerName());

    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "target_joint_states", 10,
      std::bind(&OP3Driver::callback_joint_state, this, std::placeholders::_1));

    executor_ = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    return true;
  }
  virtual bool initialize(cnoid::SimpleControllerIO * io) override
  {
    body_ = io->body();
    dt_ = io->timeStep();

    for (std::size_t idx = 0; idx < body_->numAllJoints(); ++idx) {
      cnoid::Link * link = body_->joint(idx);
      Joint joint;
      joint.id = link->jointId();
      joint.name = link->jointName();
      joint.position = link->q();
      joint_table_[link->jointName()] = joint;

      link->setActuationMode(cnoid::Link::JointDisplacement);
      io->enableIO(link);
    }

    return true;
  }
  virtual bool control() override
  {
    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      cnoid::Link * joint = body_->joint(idx);
      joint->q_target() = joint_table_[joint->name()].position;
    }
    return true;
  }

  void callback_joint_state(const sensor_msgs::msg::JointState msg)
  {
    for (int idx = 0; idx < msg.name.size(); idx++) {
      joint_table_[msg.name[idx]].position = msg.position[idx];
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  std::unique_ptr<rclcpp::executors::StaticSingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::mutex mutex_;

  std::map<std::string, Joint> joint_table_;

  cnoid::Body * body_;

  double dt_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(OP3Driver)
