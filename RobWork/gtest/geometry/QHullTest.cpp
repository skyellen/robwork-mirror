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


/* Constructs a square in 2D and considers a set of 4 points around it.
 * Checks if the correct result is gained for:
 *  - isInside,
 *  - getMinDistInside, and
 *  - getMinDistOutside.
 */
TEST(QHullND, distanceFuncs) {
	QHullND<2> qhull;
	std::vector<VectorND<2> > vertices(4); // square with corners at "1,1"
	vertices[0][0] = 1; // top right
	vertices[0][1] = 1;
	vertices[1][0] = 1; // bot right
	vertices[1][1] = -1;
	vertices[2][0] = -1; // top left
	vertices[2][1] = 1;
	vertices[3][0] = -1; // bot left
	vertices[3][1] = -1;
	qhull.rebuild(vertices);

	VectorND<2> inside_far_from_boarder, inside_close_to_boarder;
	inside_far_from_boarder[0] = 0.1;
	inside_far_from_boarder[1] = 0.3;
	inside_close_to_boarder[0] = -0.8;
	inside_close_to_boarder[1] = 0.55;
	VectorND<2> outside_far_from_boarder, outside_close_to_boarder;
	outside_far_from_boarder[0] = 3.0;
	outside_far_from_boarder[1] = 0.3;
	outside_close_to_boarder[0] = -1.3;
	outside_close_to_boarder[1] = 0.32;

	// -- test if inside/outside is right --
	EXPECT_TRUE(qhull.isInside(inside_far_from_boarder));
	EXPECT_TRUE(qhull.isInside(inside_close_to_boarder));
	EXPECT_FALSE(qhull.isInside(outside_far_from_boarder));
	EXPECT_FALSE(qhull.isInside(outside_close_to_boarder));

	// -- test outside distances --
	static const double EPSILON = 0.000000001; // own epsilon is used to account for precision in calculations...
	EXPECT_NEAR(qhull.getMinDistOutside(inside_far_from_boarder), 0.0, EPSILON);
	EXPECT_NEAR(qhull.getMinDistOutside(inside_close_to_boarder), 0.0, EPSILON);
	EXPECT_NEAR(qhull.getMinDistOutside(outside_far_from_boarder), 2.0, EPSILON);
	EXPECT_NEAR(qhull.getMinDistOutside(outside_close_to_boarder), 0.3, EPSILON);

	// -- test inside distances --
	EXPECT_NEAR(qhull.getMinDistInside(inside_far_from_boarder), 0.7, EPSILON);
	EXPECT_NEAR(qhull.getMinDistInside(inside_close_to_boarder), 0.2, EPSILON);

	// -- Warning: The following test is expected to fail due to a legacy implementation that is dared not fixed. --"
	//EXPECT_NEAR(qhull.getMinDistInside(outside_far_from_boarder), 0.0, EPSILON); // Expected to fail - Legacy implementation returns values <0 for outside

	// " -- Warning: The following test is expected to fail due to a legacy implementation that is dared not fixed. --"
	//EXPECT_NEAR(qhull.getMinDistInside(outside_close_to_boarder), 0.0, EPSILON); // Expected to fail - Legacy implementation returns values <0 for outside
}
