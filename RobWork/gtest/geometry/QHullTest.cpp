/********************************************************************************
* Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/geometry/QHullND.hpp>
#include <rw/math/VectorND.hpp>

#include <vector>
#include <cmath>
#include <limits>

using rw::geometry::QHullND;
using rw::math::VectorND;

TEST(QHullND, build2D) {
	QHullND<2> qhull;
	std::vector<VectorND<2> > vertices(3);
	vertices[0][0] = 1;
	vertices[0][1] = 0;
	vertices[1][0] = 0;
	vertices[1][1] = 1;
	vertices[2][0] = -1;
	vertices[2][1] = -1;
	qhull.rebuild(vertices);
	const VectorND<2> center = qhull.getCentroid();
	EXPECT_NEAR(std::sqrt(10./144.)-1./3., center[0], std::numeric_limits<double>::epsilon());
	EXPECT_NEAR(std::sqrt(10./144.)-1./3., center[1], std::numeric_limits<double>::epsilon());
}
