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


#ifndef RW_GEOMETRY_CONE_HPP_
#define RW_GEOMETRY_CONE_HPP_

#include "Primitive.hpp"

//! @file Cone.hpp

namespace rw {
namespace geometry {
	//! @addtogroup @{
	/**
	 * @brief cone primitive. Like a cylinder though where a radius can be
	 * specified for both ends.
	 *
	 * The cone is aligned with the z-axis such that top is in the positive z-axis
	 * and the bottom is in the negative z-axis. The center of the cone will be
	 * in (0,0,0)
	 *
	 */
	class Cone: public Primitive {
	public:
		//! @brief constructor
		Cone(const rw::math::Q& initQ);

		/**
		 * @brief constructor
		 * @param height [in] height of cone
		 * @param radiusTop [in] radius of the top end
		 * @param radiusBot [in] radius of the bottom end
		 */
		Cone(double height, double radiusTop, double radiusBot);

		//! @brief the height
		double getHeight(){ return _height;}

		//! @brief the top radius
		double getTopRadius(){ return _radiusTop;};

		//! @brief the bottom radius
		double getBottomRadius(){ return _radiusBottom;};

		//! @brief destructor
		virtual ~Cone();

		// inherited from Primitive
		//! @copydoc GeometryData::createMesh
		TriMeshPtr createMesh(int resolution) const;

		//! @copydoc GeometryData::getParameters
		rw::math::Q getParameters() const;

		//! @copydoc GeometryData::getType
		GeometryType getType(){ return ConePrim; };

	private:
		double _radiusTop, _radiusBottom, _height;
	};

	//! @}

} // geometry
} // rw

#endif /* CONE_HPP_ */
