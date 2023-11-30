// Copyright 2023 szkwiatkowski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mrac_controller/mrac_controller_node.hpp"
#include "controller_components/adaptive_controller.hpp"

namespace mrac_controller
{

MracControllerNode::MracControllerNode(const rclcpp::NodeOptions & options)
:  Node("mrac_controller", options)
{
  mrac_controller_ = std::make_unique<mrac_controller::MracController>();
  param_name_ = this->declare_parameter("param_name", 456);
  mrac_controller_->foo(param_name_);
}

}  // namespace mrac_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mrac_controller::MracControllerNode)
