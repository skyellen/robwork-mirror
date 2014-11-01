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


#ifndef RW_GEOMETRY_PYRAMID_HPP_
#define RW_GEOMETRY_PYRAMID_HPP_


#include "Primitive.hpp"

namespace rw {
namespace geometry {


    /**
     * @brief a pyrimidal geometric primitive. The pyramid has a rectangular base in the xy-plane
     * and its end pointed lie in the z-axis with a distance from the xy-plane.
     */
    class Pyramid: public Primitive {
    public:
        /**
         * @brief constructor
         */
        Pyramid(const rw::math::Q& initQ);

        /**
         * @brief constructor
         * @param widthx [in] width of pyramid in x axis
         * @param widthy [in] width of pyramid in y-axis
         * @param height [in] height of pyramid in z-axis
         * @return
         */
        Pyramid(double widthx, double widthy, double height);

        //! @brief destructor
        virtual ~Pyramid();

        // inherited from Primitive
        //! @copydoc Primitive::createMesh
		TriMesh::Ptr createMesh(int resolution) const;

        //! @copydoc Primitive::getParameters
        rw::math::Q getParameters() const;

		//! @copydoc GeometryData::getType
		GeometryType getType() const { return PyramidPrim; };

    protected:
        bool doIsInside(const rw::math::Vector3D<>& point);
    private:
        double _widthX, _widthY, _height;
    };

} // geometry
} // rw


#endif /* PYRAMID_HPP_ */
