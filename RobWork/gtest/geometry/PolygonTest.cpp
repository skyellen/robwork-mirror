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

#include <rw/geometry/Polygon.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

using rw::geometry::Polygon;
using namespace rw::math;

TEST(Polygon, Vector3D) {
	Polygon<> polygon;
	polygon.addVertex(Vector3D<>(1.,0.,-0.1));
	polygon.addVertex(Vector3D<>(100.,100.,100.));
	polygon.addVertex(Vector3D<>(-1.,1.,-0.1));
	polygon.addVertex(Vector3D<>(-1.,-1.,-0.1));
	EXPECT_EQ(4,polygon.size());
	polygon.removeVertex(1);
	EXPECT_EQ(3,polygon.size());
	EXPECT_DOUBLE_EQ(-1./3.,polygon.computeCenter()[0]);
	EXPECT_DOUBLE_EQ(0.,polygon.computeCenter()[1]);
	EXPECT_DOUBLE_EQ(-0.1,polygon.computeCenter()[2]);
	EXPECT_EQ(1.,polygon.getVertex(0)[0]);
	EXPECT_EQ(0.,polygon.getVertex(0)[1]);
	EXPECT_EQ(-0.1,polygon.getVertex(0)[2]);
	EXPECT_EQ(-1.,polygon[2][0]);
	EXPECT_EQ(-1.,polygon[2][1]);
	EXPECT_EQ(-0.1,polygon[2][2]);
}

TEST(Polygon, Vector2D) {
	Polygon<Vector2D<> > polygon;
	polygon.addVertex(Vector2D<>(1.,0.));
	polygon.addVertex(Vector2D<>(100.,100.));
	polygon.addVertex(Vector2D<>(-1.,1.));
	polygon.addVertex(Vector2D<>(-1.,-1.));
	EXPECT_EQ(4,polygon.size());
	polygon.removeVertex(1);
	EXPECT_EQ(3,polygon.size());
	EXPECT_DOUBLE_EQ(-1./3.,polygon.computeCenter()[0]);
	EXPECT_DOUBLE_EQ(0.,polygon.computeCenter()[1]);
	EXPECT_EQ(1.,polygon.getVertex(0)[0]);
	EXPECT_EQ(0.,polygon.getVertex(0)[1]);
	EXPECT_EQ(-1.,polygon[2][0]);
	EXPECT_EQ(-1.,polygon[2][1]);
}

TEST(DeathTest, Polygon) {
	::testing::FLAGS_gtest_death_test_style = "threadsafe";
	Polygon<> polygon;
	polygon.addVertex(Vector3D<>(1.,0.,-0.1));
	polygon.addVertex(Vector3D<>(100.,100.,100.));
	polygon.addVertex(Vector3D<>(-1.,1.,-0.1));
	polygon.addVertex(Vector3D<>(-1.,-1.,-0.1));
	EXPECT_DEATH_IF_SUPPORTED(polygon.removeVertex(4),"Polygon.hpp:*");
	EXPECT_DEATH_IF_SUPPORTED(polygon.getVertex(4),"Polygon.hpp:*");
}
