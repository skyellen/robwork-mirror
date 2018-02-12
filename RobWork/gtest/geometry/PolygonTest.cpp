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
#include <rw/geometry/PolygonUtil.hpp>
#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>

using rw::geometry::Polygon;
using rw::geometry::PolygonUtil;
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

TEST(PolygonUtil, ConvexDecomposition2D) {
	Polygon<Vector2D<> > polygon;
	polygon.addVertex(Vector2D<>(-0.05, 0.1));
	polygon.addVertex(Vector2D<>(0.2, 0.2));
	polygon.addVertex(Vector2D<>(0.35, 0.07));
	polygon.addVertex(Vector2D<>(0.5, 0.23));
	polygon.addVertex(Vector2D<>(0.7, 0.26));
	polygon.addVertex(Vector2D<>(0.7, 0.035));
	polygon.addVertex(Vector2D<>(0.4, -0.05));
	polygon.addVertex(Vector2D<>(0.55, -0.15));
	polygon.addVertex(Vector2D<>(0.68, -0.17));
	polygon.addVertex(Vector2D<>(0.45, -0.4));
	polygon.addVertex(Vector2D<>(0.275, -0.2));
	polygon.addVertex(Vector2D<>(0, -0.16));

	const std::vector<Polygon<Vector2D<> > > dec = PolygonUtil::convexDecomposition(polygon);
	EXPECT_EQ(4,dec.size());
	double subPolyAreaTotal = 0;
	for (std::size_t i = 0; i < dec.size(); i++) {
		const Polygon<Vector2D<> >& p = dec[i];
		subPolyAreaTotal += PolygonUtil::area(p);
	}
	EXPECT_DOUBLE_EQ(PolygonUtil::area(polygon),subPolyAreaTotal);
}

TEST(PolygonUtil, Area) {
	Polygon<Vector2D<> > polygonCW;
	polygonCW.addVertex(Vector2D<>(0.55, -0.15));
	polygonCW.addVertex(Vector2D<>(0.68, -0.17));
	polygonCW.addVertex(Vector2D<>(0.45, -0.4));
	polygonCW.addVertex(Vector2D<>(0.275, -0.2));

	Polygon<Vector2D<> > polygonCCW;
	polygonCCW.addVertex(Vector2D<>(0.275, -0.2));
	polygonCCW.addVertex(Vector2D<>(0.45, -0.4));
	polygonCCW.addVertex(Vector2D<>(0.68, -0.17));
	polygonCCW.addVertex(Vector2D<>(0.55, -0.15));

	EXPECT_DOUBLE_EQ(-0.049125,PolygonUtil::area(polygonCW));
	EXPECT_DOUBLE_EQ(0.049125,PolygonUtil::area(polygonCCW));
}

TEST(PolygonUtil, IsInside) {
	Polygon<Vector2D<> > polygon;
	polygon.addVertex(Vector2D<>(0.55, -0.15));
	polygon.addVertex(Vector2D<>(0.68, -0.17));
	polygon.addVertex(Vector2D<>(0.45, -0.4));
	polygon.addVertex(Vector2D<>(0.275, -0.2));

	EXPECT_TRUE(PolygonUtil::isInsideConvex(Vector2D<>(0.4,-0.3),polygon,std::numeric_limits<double>::epsilon()*5));
	EXPECT_FALSE(PolygonUtil::isInsideConvex(Vector2D<>(0.29,-0.35),polygon,std::numeric_limits<double>::epsilon()*5));
	EXPECT_FALSE(PolygonUtil::isInsideConvex(Vector2D<>(0.45,-0.4),polygon,std::numeric_limits<double>::epsilon()*5));
}
