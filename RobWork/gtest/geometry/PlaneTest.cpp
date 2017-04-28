/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include <gtest/gtest.h>

#include <rw/geometry/Plane.hpp>

using rw::geometry::Plane;
using rw::math::Vector3D;

TEST(PlaneTest, intersection) {
	const Plane plane(normalize(Vector3D<>(-1,2,0)),-0.1);
	const Vector3D<> intersection = plane.intersection(Vector3D<>(1,0,0), Vector3D<>(1,1,0));
	EXPECT_DOUBLE_EQ(1.,intersection[0]);
	EXPECT_DOUBLE_EQ(0.5+0.1/std::cos(std::atan(1./2.)),intersection[1]);
	EXPECT_DOUBLE_EQ(0.,intersection[2]);
}
