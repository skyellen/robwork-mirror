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
using namespace rw::math;

TEST(PlaneTest, basics) {
	static const Vector3D<> n = normalize(Vector3D<>(-1,2,0));
	{
		static const Plane plane;
		EXPECT_DOUBLE_EQ(0.,plane.normal()[0]);
		EXPECT_DOUBLE_EQ(0.,plane.normal()[1]);
		EXPECT_DOUBLE_EQ(1.,plane.normal()[2]);
		EXPECT_DOUBLE_EQ(0.,plane.d());
		EXPECT_DOUBLE_EQ(0.,plane.distance(Vector3D<>::zero()));
	}
	{
		static const Plane plane(Q(4,n[0],n[1],n[2],4.));
		EXPECT_DOUBLE_EQ(n[0],plane.normal()[0]);
		EXPECT_DOUBLE_EQ(n[1],plane.normal()[1]);
		EXPECT_DOUBLE_EQ(n[2],plane.normal()[2]);
		EXPECT_DOUBLE_EQ(4.,plane.d());
		EXPECT_DOUBLE_EQ(4.,plane.distance(Vector3D<>::zero()));
	}
	{
		static const Plane plane(n,4.);
		EXPECT_DOUBLE_EQ(n[0],plane.normal()[0]);
		EXPECT_DOUBLE_EQ(n[1],plane.normal()[1]);
		EXPECT_DOUBLE_EQ(n[2],plane.normal()[2]);
		EXPECT_DOUBLE_EQ(4.,plane.d());
		EXPECT_DOUBLE_EQ(4.,plane.distance(Vector3D<>::zero()));
	}
	{
		static const double d = 0.1;
		static const Vector3D<> dir = normalize(Vector3D<>(2,1,0));
		static const Vector3D<> p1 = -d*n+0.1*dir;
		static const Vector3D<> p2 = -d*n+0.1*cross(n,dir);
		static const Vector3D<> p3 = -d*n-0.1*dir;
		static const Plane plane(p1,p2,p3);
		EXPECT_DOUBLE_EQ(n[0],plane.normal()[0]);
		EXPECT_DOUBLE_EQ(n[1],plane.normal()[1]);
		EXPECT_NEAR(n[2],plane.normal()[2],std::numeric_limits<double>::epsilon());
		EXPECT_NEAR(d,plane.d(),1e-15);
		EXPECT_DOUBLE_EQ(d,plane.distance(Vector3D<>::zero()));
		EXPECT_DOUBLE_EQ(0.,plane.distance(p1));
		EXPECT_DOUBLE_EQ(0.,plane.distance(p2));
		EXPECT_DOUBLE_EQ(0.,plane.distance(p3));
	}
}

TEST(PlaneTest, refit) {
	std::vector<Vector3D<> > data;
	data.push_back(Vector3D<>::x());
	data.push_back(Vector3D<>::y());
	data.push_back(Vector3D<>::z());
	Plane plane;
	EXPECT_DOUBLE_EQ(0.,plane.refit(data));
	EXPECT_DOUBLE_EQ(1./std::sqrt(3.),plane.normal()[0]);
	EXPECT_DOUBLE_EQ(1./std::sqrt(3.),plane.normal()[1]);
	EXPECT_DOUBLE_EQ(1./std::sqrt(3.),plane.normal()[2]);
	EXPECT_DOUBLE_EQ(-1./std::sqrt(3.),plane.d());
	EXPECT_DOUBLE_EQ(-1./std::sqrt(3.),plane.distance(Vector3D<>::zero()));
}

TEST(PlaneTest, intersection) {
	const Plane plane(normalize(Vector3D<>(-1,2,0)),-0.1);
	const Vector3D<> intersection = plane.intersection(Vector3D<>(1,0,0), Vector3D<>(1,1,0));
	EXPECT_DOUBLE_EQ(1.,intersection[0]);
	EXPECT_DOUBLE_EQ(0.5+0.1/std::cos(std::atan(1./2.)),intersection[1]);
	EXPECT_DOUBLE_EQ(0.,intersection[2]);
}
