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

#ifndef MRAC_CONTROLLER__MRAC_CONTROLLER_HPP_
#define MRAC_CONTROLLER__MRAC_CONTROLLER_HPP_

#include <cstdint>

#include "mrac_controller/visibility_control.hpp"


namespace mrac_controller
{

class MRAC_CONTROLLER_PUBLIC MracController
{
public:
  MracController();
  int64_t foo(int64_t bar) const;
};

}  // namespace mrac_controller

#endif  // MRAC_CONTROLLER__MRAC_CONTROLLER_HPP_
