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

#include <rw/geometry/IntersectUtil.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/geometry/Plane.hpp>

using namespace rw::geometry;
using rw::math::Vector3D;

TEST(IntersectUtilTest, closestPoint_PointLineSegment) {
	static const Vector3D<> ray1(0,1,1);
	static const Vector3D<> ray2(5,5,-1);
	{
		static const Vector3D<> p(6.5,-2,0);
		// Between the two points
		Vector3D<> point;
		point = IntersectUtil::closestPt(p,Line(ray1,ray2));
		EXPECT_DOUBLE_EQ(2.5,point[0]);
		EXPECT_DOUBLE_EQ(3.,point[1]);
		EXPECT_DOUBLE_EQ(0.,point[2]);
		point = IntersectUtil::closestPtPointLine(p,ray1,ray2);
		EXPECT_DOUBLE_EQ(2.5,point[0]);
		EXPECT_DOUBLE_EQ(3.,point[1]);
		EXPECT_DOUBLE_EQ(0.,point[2]);
	}
	{
		static const Vector3D<> p(-1,-8,3);
		// Before first point
		Vector3D<> point;
		point = IntersectUtil::closestPt(p,Line(ray1,ray2));
		EXPECT_DOUBLE_EQ(0,point[0]);
		EXPECT_DOUBLE_EQ(1,point[1]);
		EXPECT_DOUBLE_EQ(1,point[2]);
		point = IntersectUtil::closestPtPointLine(p,ray1,ray2);
		EXPECT_DOUBLE_EQ(0,point[0]);
		EXPECT_DOUBLE_EQ(1,point[1]);
		EXPECT_DOUBLE_EQ(1,point[2]);
	}
	{
		static const Vector3D<> p(14,4,-3);
		// After second point
		Vector3D<> point;
		point = IntersectUtil::closestPt(p,Line(ray1,ray2));
		EXPECT_DOUBLE_EQ(5,point[0]);
		EXPECT_DOUBLE_EQ(5,point[1]);
		EXPECT_DOUBLE_EQ(-1,point[2]);
		point = IntersectUtil::closestPtPointLine(p,ray1,ray2);
		EXPECT_DOUBLE_EQ(5,point[0]);
		EXPECT_DOUBLE_EQ(5,point[1]);
		EXPECT_DOUBLE_EQ(-1,point[2]);
	}
}

TEST(IntersectUtilTest, closestPoint_PointRay) {
	static const Vector3D<> ray1(0,1,1);
	static const Vector3D<> ray2(5,5,-1);
	{
		// Between the two points
		const Vector3D<> point = IntersectUtil::closestPtPointRay(Vector3D<>(6.5,-2,0),ray1,ray2);
		EXPECT_DOUBLE_EQ(2.5,point[0]);
		EXPECT_DOUBLE_EQ(3.,point[1]);
		EXPECT_DOUBLE_EQ(0.,point[2]);
	}
	{
		// Before first point
		const Vector3D<> point = IntersectUtil::closestPtPointRay(Vector3D<>(-1,-8,3),ray1,ray2);
		EXPECT_DOUBLE_EQ(-5,point[0]);
		EXPECT_DOUBLE_EQ(-3,point[1]);
		EXPECT_DOUBLE_EQ(3,point[2]);
	}
	{
		// After second point
		const Vector3D<> point = IntersectUtil::closestPtPointRay(Vector3D<>(14,4,-3),ray1,ray2);
		EXPECT_DOUBLE_EQ(10,point[0]);
		EXPECT_DOUBLE_EQ(9,point[1]);
		EXPECT_DOUBLE_EQ(-3,point[2]);
	}
}

TEST(IntersectUtilTest, intersectionPoint_RayPlane) {
	static const Vector3D<> ray1(0,1,1);
	static const Vector3D<> ray2(5,5,-1);
	{
		Vector3D<> dst;
		EXPECT_TRUE(IntersectUtil::intersetPtRayPlane(ray1,ray2,Plane(Vector3D<>::x(),-2.5),dst));
		EXPECT_DOUBLE_EQ(2.5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0,dst[2]);
		EXPECT_FALSE(IntersectUtil::intersetPtRayPlane(ray1,ray2,Plane(normalize(Vector3D<>(4,-5,0)),0),dst));
	}
	{
		Vector3D<> dst;
		EXPECT_TRUE(IntersectUtil::intersetPtRayPlane(ray1,ray2,Vector3D<>(2.5,0,0),Vector3D<>(2.5,0,-1),Vector3D<>(2.5,1,0),dst));
		EXPECT_DOUBLE_EQ(2.5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0,dst[2]);
		EXPECT_FALSE(IntersectUtil::intersetPtRayPlane(ray1,ray2,ray1,ray2,Vector3D<>(2.5,3,-5),dst));
	}
}

TEST(IntersectUtilTest, intersectionPoint_RayTriangle) {
	static const Vector3D<> p1(2,1,-2);
	static const Vector3D<> p2(7,2,2);
	static const Vector3D<> p3(6,5,2);
	{
		static const Vector3D<> ray1(5,3,10);
		static const Vector3D<> ray2(5,3,5);
		Vector3D<> dst;
		EXPECT_TRUE(IntersectUtil::intersetPtRayTri(ray1,ray2,Triangle<>(p1,p2,p3),dst));
		EXPECT_DOUBLE_EQ(5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0.75,dst[2]);
		EXPECT_TRUE(IntersectUtil::intersetPtRayTri(ray1,ray2,p1,p2,p3,dst));
		EXPECT_DOUBLE_EQ(5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0.75,dst[2]);
	}
	{
		Vector3D<> dst;
		Vector3D<> vec;
		vec = Vector3D<>(2,3,0);
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
		vec = Vector3D<>(6.5,4,0);
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
		vec = Vector3D<>(4,1,0);
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtRayTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
	}
}

TEST(IntersectUtilTest, intersectionPoint_LinePlane) {
	static const Vector3D<> n = normalize(Vector3D<>(-1,2,0));
	static const double d = -0.1;
	static const Vector3D<> p1 = -d*n;
	static const Vector3D<> p2 = -d*n+Vector3D<>(2,1,0);
	static const Vector3D<> p3 = -d*n+Vector3D<>(2,1,-1);
	{
		static const Vector3D<> line1(1,0,0);
		static const Vector3D<> line2(1,1,0);
		Vector3D<> dst;
		EXPECT_TRUE(IntersectUtil::intersetPtLinePlane(line1,line2,Plane(n,d),dst));
		EXPECT_DOUBLE_EQ(1.,dst[0]);
		EXPECT_DOUBLE_EQ(0.5+0.1/std::cos(std::atan(1./2.)),dst[1]);
		EXPECT_DOUBLE_EQ(0.,dst[2]);
		EXPECT_TRUE(IntersectUtil::intersetPtLinePlane(line1,line2,p1,p2,p3,dst));
		EXPECT_DOUBLE_EQ(1.,dst[0]);
		EXPECT_DOUBLE_EQ(0.5+0.1/std::cos(std::atan(1./2.)),dst[1]);
		EXPECT_DOUBLE_EQ(0.,dst[2]);
	}
	{
		static const Vector3D<> line1(0,0,0);
		static const Vector3D<> line2(2,1,0);
		Vector3D<> dst;
		EXPECT_FALSE(IntersectUtil::intersetPtLinePlane(line1,line2,Plane(n,d),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtLinePlane(line1,line2,p1,p2,p3,dst));
	}
}

TEST(IntersectUtilTest, intersectionPoint_LineTriangle) {
	static const Vector3D<> p1(2,1,-2);
	static const Vector3D<> p2(7,2,2);
	static const Vector3D<> p3(6,5,2);
	{
		static const Vector3D<> ray1(5,3,-5);
		static const Vector3D<> ray2(5,3,5);
		Vector3D<> dst;
		EXPECT_TRUE(IntersectUtil::intersetPtLineTri(ray1,ray2,Triangle<>(p1,p2,p3),dst));
		EXPECT_DOUBLE_EQ(5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0.75,dst[2]);
		EXPECT_TRUE(IntersectUtil::intersetPtLineTri(ray1,ray2,p1,p2,p3,dst));
		EXPECT_DOUBLE_EQ(5,dst[0]);
		EXPECT_DOUBLE_EQ(3,dst[1]);
		EXPECT_DOUBLE_EQ(0.75,dst[2]);
	}
	// Line segment too short
	{
		Vector3D<> dst;
		Vector3D<> ray1;
		Vector3D<> ray2;
		ray1 = Vector3D<>(5,3,10);
		ray2 = Vector3D<>(5,3,5);
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(ray1,ray2,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(ray1,ray2,p1,p2,p3,dst));
	}
	// Ray outside
	{
		Vector3D<> dst;
		Vector3D<> vec;
		vec = Vector3D<>(2,3,0);
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
		vec = Vector3D<>(6.5,4,0);
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
		vec = Vector3D<>(4,1,0);
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,Triangle<>(p1,p2,p3),dst));
		EXPECT_FALSE(IntersectUtil::intersetPtLineTri(vec+Vector3D<>::z(),vec,p1,p2,p3,dst));
	}
}

TEST(IntersectUtilTest, intersectionPoint_TriangleTriangle) {
	static const Vector3D<> pa1(2,1,-2);
	static const Vector3D<> pa2(7,2,2);
	static const Vector3D<> pa3(7,5,2);
	{
		static const Vector3D<> pb1(1,3,2);
		static const Vector3D<> pb2(6,1,0);
		static const Vector3D<> pb3(6,6,0);
		Vector3D<> dst1;
		Vector3D<> dst2;
		EXPECT_TRUE(IntersectUtil::intersetPtTriTri(Triangle<>(pa1,pa2,pa3),Triangle<>(pb1,pb2,pb3),dst1,dst2));
		EXPECT_DOUBLE_EQ(5,dst1[0]);
		EXPECT_DOUBLE_EQ(1.6,dst1[1]);
		EXPECT_DOUBLE_EQ(0.4,dst1[2]);
		EXPECT_DOUBLE_EQ(5,dst2[0]);
		EXPECT_DOUBLE_EQ(3.4,dst2[1]);
		EXPECT_DOUBLE_EQ(0.4,dst2[2]);
	}
	{
		static const Vector3D<> pb1(1,3,2);
		static const Vector3D<> pb2(6,1,2);
		static const Vector3D<> pb3(6,6,2);
		Vector3D<> dst1;
		Vector3D<> dst2;
		EXPECT_FALSE(IntersectUtil::intersetPtTriTri(Triangle<>(pa1,pa2,pa3),Triangle<>(pb1,pb2,pb3),dst1,dst2));
	}
}
