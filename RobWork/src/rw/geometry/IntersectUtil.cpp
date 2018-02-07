/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "IntersectUtil.hpp"

#include "Plane.hpp"
#include "Line.hpp"

#include <rw/math/Math.hpp>

using namespace rw::geometry;
using namespace rw::math;

rw::math::Vector3D<> IntersectUtil::closestPt(const Vector3D<>& point, const Line& line)
{
	return closestPtPointLine(point, line.p1(), line.p2());
}


/*
rw::math::Vector3D<> IntersectUtil::closestPtPointRay(const Vector3D<>& point, const Vector3D<>& ray)
{
	
}

IntersectUtil::PointPair IntersectUtil::closestPts(const Line& lineA, const Line& lineB){

}

IntersectUtil::PointPair IntersectUtil::closestPts(const Line& line, const Triangle<>& tri){

}
*/
rw::math::Vector3D<> IntersectUtil::closestPtPointRay(const rw::math::Vector3D<>& point,
										    const rw::math::Vector3D<>& p1,
											const rw::math::Vector3D<>& p2)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	double t = dot(point - p1,ab)/dot(ab,ab);
	// if outside segment, clamp t
	//t = Math::clamp(t,0.0,1.0);
	// compute projected position
	return p1 + t*ab;
}

rw::math::Vector3D<> IntersectUtil::closestPtPointLine(const rw::math::Vector3D<>& point,
										    const rw::math::Vector3D<>& p1,
											const rw::math::Vector3D<>& p2)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	double t = dot(point - p1,ab)/dot(ab,ab);
	// if outside segment, clamp t
	t = Math::clamp(t,0.0,1.0);
	// compute projected position
	return p1 + t*ab;
}


/*IntersectUtil::PointPair IntersectUtil::closestPtsRayTraingle(
		const rw::math::Vector3D<>& ray,
		const Triangle<>& tri)
{
	// for each segment in the triangle we do a closest point calculation

}*/

bool IntersectUtil::intersetPtRayPlane(
		const rw::math::Vector3D<> &p1,
		const rw::math::Vector3D<> &p2,
		const Plane& p, Vector3D<> &dst)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	//double t = (p.d() - dot(p.normal() - p1, p1)) / dot(p.normal(),ab);
	double t = (-p.d() - dot(p.normal(), p1)) / dot(p.normal(),ab);

	// if abs(t) is very large then the ray is parallel to plane
	if(10000000<fabs(t) || Math::isNaN(t))
		return false;
	// compute projected position
	dst = p1 + t*ab;
	return true;
}

bool IntersectUtil::intersetPtRayPlane(
						const rw::math::Vector3D<>& rayp1,
						const rw::math::Vector3D<>& rayp2,
						const rw::math::Vector3D<>& p1,
						const rw::math::Vector3D<>& p2,
						const rw::math::Vector3D<>& p3,
						rw::math::Vector3D<>& dst)
{
	Plane p(p1, p2, p3);
	return IntersectUtil::intersetPtRayPlane(rayp1, rayp2, p, dst);
}

bool IntersectUtil::intersetPtRayTri(
								const rw::math::Vector3D<>& p1,
								const rw::math::Vector3D<>& p2,
								const Triangle<>& tri,
								rw::math::Vector3D<>& dst)
{
	return intersetPtRayTri(p1,p2,tri[0],tri[1],tri[2],dst);
}


bool IntersectUtil::intersetPtRayTri(const rw::math::Vector3D<>& ray1,
												const rw::math::Vector3D<>& ray2,
												const rw::math::Vector3D<>& p1,
												const rw::math::Vector3D<>& p2,
												const rw::math::Vector3D<>& p3,
												rw::math::Vector3D<>& dst)
{
	Vector3D<> tmp_dst;
	Plane p(p1, p2, p3);
	// test if ray intersects the supporting plane of the triangle
	if(!intersetPtRayPlane(ray1,ray2,p,tmp_dst))
		return false;
	// next we need to determine if intersection point is inside triangle
	if( dot(cross((p2-p1),(tmp_dst-p1)), p.normal() ) <0 )
		return false;
	if( dot(cross((p3-p2),(tmp_dst-p2)), p.normal() ) <0 )
		return false;
	if( dot(cross((p1-p3),(tmp_dst-p3)), p.normal() ) <0 )
		return false;
	dst = tmp_dst;
	return true;
}


bool IntersectUtil::intersetPtLinePlane(
		const rw::math::Vector3D<> &p1,
		const rw::math::Vector3D<> &p2,
		const Plane& p, Vector3D<> &dst)
{
	Vector3D<> ab = p2-p1;
	// project c onto ab, computing parameterizated pos
	//double t = (p.d() - dot(p.normal() - p1, p1)) / dot(p.normal(),ab);
	double t = (-p.d() - dot(p.normal(), p1)) / dot(p.normal(),ab);

	// if abs(t) is very large then the ray is parallel to plane
	if(10000000<fabs(t))
		return false;

	// if t is larger than 1 then the intersection point is outside the line segment
	//std::cout << "t:" << t << std::endl;
	if(fabs(t)>1.0)
		return false;

	// compute projected position
	dst = p1 + t*ab;
	return true;
}

bool IntersectUtil::intersetPtLinePlane(
						const rw::math::Vector3D<>& rayp1,
						const rw::math::Vector3D<>& rayp2,
						const rw::math::Vector3D<>& p1,
						const rw::math::Vector3D<>& p2,
						const rw::math::Vector3D<>& p3,
						rw::math::Vector3D<>& dst)
{
	Plane p(p1, p2, p3);
	return IntersectUtil::intersetPtLinePlane(rayp1, rayp2, p, dst);
}


bool IntersectUtil::intersetPtLineTri(const rw::math::Vector3D<>& ray1,
						const rw::math::Vector3D<>& ray2,
						const rw::math::Vector3D<>& p1,
						const rw::math::Vector3D<>& p2,
						const rw::math::Vector3D<>& p3,
						rw::math::Vector3D<>& dst)
{
	rw::math::Vector3D<> tmp_dst;
	Plane p(p1, p2, p3);
	// test if ray intersects the supporting plane of the triangle
	if(!intersetPtLinePlane(ray1,ray2,p,tmp_dst))
		return false;
	// next we need to determine if intersection point is inside triangle
	if( dot(cross((p2-p1),(tmp_dst-p1)), p.normal() ) <0 )
		return false;
	if( dot(cross((p3-p2),(tmp_dst-p2)), p.normal() ) <0 )
		return false;
	if( dot(cross((p1-p3),(tmp_dst-p3)), p.normal() ) <0 )
		return false;
	dst = tmp_dst;
	return true;
}

bool IntersectUtil::intersetPtLineTri(
						const rw::math::Vector3D<>& p1,
						const rw::math::Vector3D<>& p2,
						const Triangle<>& tri,
						rw::math::Vector3D<>& dst)
{
	return intersetPtLineTri(p1,p2,tri[0],tri[1],tri[2],dst);
}

bool IntersectUtil::intersetPtTriTri(
						const Triangle<>& triA,
						const Triangle<>& triB,
						rw::math::Vector3D<>& dst1,
						rw::math::Vector3D<>& dst2)
{
/*
	std::cout << "triA\n"
			  << "\t" << triA[0] << "\n"
			  << "\t" << triA[1] << "\n"
			  << "\t" << triA[2] << "\n";
	std::cout << "triB\n"
			  << "\t" << triB[0] << "\n"
			  << "\t" << triB[1] << "\n"
			  << "\t" << triB[2] << "\n";
*/
	// we test if each line segment of triA intersects triB and vice versa
	rw::math::Vector3D<> *dst = &dst1;
	if(intersetPtLineTri(triA[0],triA[1], triB, *dst))
		dst = &dst2;
	if(intersetPtLineTri(triA[1],triA[2], triB, *dst))
		dst = &dst2;
	if(intersetPtLineTri(triA[2],triA[0], triB, *dst)){
		if(dst==&dst2)
			return true;
		dst = &dst2;
	}

	if(intersetPtLineTri(triB[0],triB[1], triA, *dst)){
		if(dst==&dst2)
			return true;
		dst = &dst2;
	}
	if(intersetPtLineTri(triB[1],triB[2], triA, *dst)){
		if(dst==&dst2)
			return true;
		dst = &dst2;
	}
	if(intersetPtLineTri(triB[2],triB[0], triA, *dst)){
		if(dst==&dst2)
			return true;
	}
	return false;
}

