// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <vector>

#include "gtest/gtest.h"
#include "rttest/math_utils.hpp"

TEST(MathUtils, calculate_stddev) {
  std::vector<uint64_t> v_int = {1, 2, 3, 4};
  EXPECT_NEAR(1.118, calculate_stddev(v_int), 0.001);

  std::vector<double> v_double = {1.1, 2.2, 3.3, 4.9};
  EXPECT_NEAR(1.404, calculate_stddev(v_double), 0.001);
}
