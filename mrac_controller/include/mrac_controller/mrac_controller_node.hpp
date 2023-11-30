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

#ifndef MRAC_CONTROLLER__MRAC_CONTROLLER_NODE_HPP_
#define MRAC_CONTROLLER__MRAC_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mrac_controller/mrac_controller.hpp"

namespace mrac_controller
{
using MracControllerPtr = std::unique_ptr<mrac_controller::MracController>;

class MRAC_CONTROLLER_PUBLIC MracControllerNode : public rclcpp::Node
{
public:
  explicit MracControllerNode(const rclcpp::NodeOptions & options);

private:
  MracControllerPtr mrac_controller_{nullptr};
  int64_t param_name_{123};
};
}  // namespace mrac_controller

#endif  // MRAC_CONTROLLER__MRAC_CONTROLLER_NODE_HPP_
