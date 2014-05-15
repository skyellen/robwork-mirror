/**
 * @file Alignment.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <string>
#include <vector>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
 


class Alignment
{
	public:
		/* constructors */
		/// Constructor
		Alignment(rw::math::Transform3D<> pose, rw::math::Q dist=rw::math::Q(5, 10.0, 10.0, 10.0, 360.0, 360.0)) :
			pose(pose),
			dist(dist)
		{}
		
		/// Destructor
		~Alignment() {}
		
		/* data */
		rw::math::Transform3D<> pose;
		rw::math::Q dist;
};
