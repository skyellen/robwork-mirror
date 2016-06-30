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
 
#ifndef _RWLIBS_CSG_CSGMODELFACTORY_HPP
#define _RWLIBS_CSG_CSGMODELFACTORY_HPP

#include <rw/math/Vector3D.hpp>

#include "CSGModel.hpp"



namespace rwlibs {
namespace csg {
	
/**
 * @brief Factory class that allows construction of CSG model primitives.
 */
class CSGModelFactory
{
public:

	/**
	 * @brief Constructs a CSGModel with a cube primitive.
	 * @param x, y, z -- length, width and depth
	 */
	static CSGModel::Ptr makeBox(float x, float y, float z);
	
	/**
	 * @brief Constructs a CSGModel with a cylinder primitive.
	 * @param r, h -- radius, height
	 */
	static CSGModel::Ptr makeCylinder(float r, float h);
	
	/**
	 * @brief Constructs a CSGModel with a sphere primitive.
	 * @param r radius
	 */
	static CSGModel::Ptr makeSphere(float r);
	
	/**
	 * @brief Constructs plane. \n
	 * Creates fake half-space, approximated by a large cube.
	 * @param point, normal point and normal
	 */
	static CSGModel::Ptr makePlane(const rw::math::Vector3D<>& point=rw::math::Vector3D<>(), const rw::math::Vector3D<>& normal=rw::math::Vector3D<>::z());
	
	/**
	 * @brief Constructs wedge. \n
	 * The wedge tip is located at origin and the edge runs along z axis.
	 * The wedge axis runs along x axis
	 */
	static CSGModel::Ptr makeWedge(float angle);
};

} /* csg */
} /* rwlibs */

#endif
