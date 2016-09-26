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

#ifndef RW_GEOMETRY_PRIMITIVE_HPP_
#define RW_GEOMETRY_PRIMITIVE_HPP_

#include "GeometryData.hpp"
#include "TriMesh.hpp"
#include <rw/math/Q.hpp>

//! @file Box.hpp

namespace rw {
namespace geometry {

//! @addtogroup geometry
// @{


    /**
     * @brief defines an interface for a geometric shape that is defined
     * by a set of parameters.
     */
    class Primitive: public GeometryData {
	public:
		//! Smart pointer to this type of class.
		typedef rw::common::Ptr<Primitive> Ptr;
		
    public:

        //! @brief destructor
        virtual ~Primitive(){};

    	/**
    	 * @copydoc GeometryData::getTriMesh
    	 * @note primitives allways return a new trimesh
    	 */
		TriMesh::Ptr getTriMesh(bool forceCopy=true) {
    		return createMesh(20);
    	}

    	/**
    	 * @brief make a trimesh from this primitive. Use \b granularity to
    	 * specify minimum number of line segments a half circle is split into
    	 * @param resolution [in]
    	 */
		virtual TriMesh::Ptr createMesh(int resolution) const = 0;

        /**
         * @brief get the parameters that define this primitive
         */
        virtual rw::math::Q getParameters() const = 0;
        
        /**
         * @brief set the parameters that define this primitive
         */
        virtual void setParameters(const rw::math::Q& q) = 0;

        //! @copydoc GeometryData::isConvex
        virtual bool isConvex() { return true; }

        //! test if a point is on the border or inside this primitive
        bool isInside(const rw::math::Vector3D<>& point ) { return doIsInside(point); };
    protected:
		/**
		 * @brief Check if point lies inside geometry.
		 * @param point [in] point to check.
		 * @return true if inside geometry, false otherwise.
		 */
        virtual bool doIsInside(const rw::math::Vector3D<>& point)
        {
            RW_THROW("Current primitive " << GeometryData::toString(getType()) << " does not implement isInside method!" );
            return false;
        }

        //! @brief Constructor.
        Primitive(){}

    };
    //! @}
}
}

#endif /* PRIMITIVE_HPP_ */
