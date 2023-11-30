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

#ifndef MRAC__MRAC_NODE_HPP_
#define MRAC__MRAC_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mrac/mrac.hpp"

namespace mrac
{
using MracPtr = std::unique_ptr<mrac::Mrac>;

class MRAC_PUBLIC MracNode : public rclcpp::Node
{
public:
  explicit MracNode(const rclcpp::NodeOptions & options);

private:
  MracPtr mrac_{nullptr};
  int64_t param_name_{123};
};
}  // namespace mrac

#endif  // MRAC__MRAC_NODE_HPP_
