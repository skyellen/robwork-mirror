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


#ifndef RW_GEOMETRY_FACE_HPP
#define RW_GEOMETRY_FACE_HPP

/**
 * @file Face.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace geometry {

    /**
     * @brief A face structure
     */
    template<class T = double>
    struct Face {
        /** @brief The normal to the Face */
        T _normal[3];
        /** @brief First vertex of the Face */
        T _vertex1[3];
        /** @brief Second vertex of the Face */
        T _vertex2[3];
        /** @brief Third vertex of the Face */
        T _vertex3[3];

        /**
         * @brief default constructor
         */
        Face(){};

        /**
         * @brief copy constructor
         *
         * @param f [in] - The face that is to be copied.
         */
        template<class A>
        Face(Face<A> f)
        {
            _normal[0] = (T)f._normal[0];
            _normal[1] = (T)f._normal[1];
            _normal[2] = (T)f._normal[2];
            _vertex1[0] = (T)f._vertex1[0];
            _vertex1[1] = (T)f._vertex1[1];
            _vertex1[2] = (T)f._vertex1[2];
            _vertex2[0] = (T)f._vertex2[0];
            _vertex2[1] = (T)f._vertex2[1];
            _vertex2[2] = (T)f._vertex2[2];
            _vertex3[0] = (T)f._vertex3[0];
            _vertex3[1] = (T)f._vertex3[1];
            _vertex3[2] = (T)f._vertex3[2];
        };

        /**
         * @brief Construct a Face from 3 rw::math::Vector3D objects
         * @param p1 [in] point 1
         * @param p2 [in] point 2
         * @param p3 [in] point 3
         */
        template<class A>
        Face(const rw::math::Vector3D<A>& p1,
             const rw::math::Vector3D<A>& p2,
             const rw::math::Vector3D<A>& p3)
        {
            rw::math::Vector3D<A> n = cross(rw::math::Vector3D<A>(p2-p1), rw::math::Vector3D<A>(p3-p1));
            n = normalize(n);
            for (int i = 0; i<3; i++) {
                _vertex1[i] = p1[i];
                _vertex2[i] = p2[i];
                _vertex3[i] = p3[i];
                _normal[i] = n[i];
            }
        }

        /**
         * @brief Construct a Face from 4 rw::math::Vector3D objects
         * @param p1 [in] point 1
         * @param p2 [in] point 2
         * @param p3 [in] point 3
         * @param n [in] face normal
         */
        template<class A>
        Face(const rw::math::Vector3D<A>& p1,
             const rw::math::Vector3D<A>& p2,
             const rw::math::Vector3D<A>& p3,
             const rw::math::Vector3D<A>& n)
        {
            for (int i = 0; i<3; i++) {
                _vertex1[i] = p1[i];
                _vertex2[i] = p2[i];
                _vertex3[i] = p3[i];
                _normal[i] = n[i];
            }
        }

        /**
         * @brief Returns Face transformed by t3d.
         */
        Face<T> transform(const rw::math::Transform3D<T>& t3d) const {
            rw::math::Vector3D<T> v1,v2,v3,n;
            for (int i = 0; i<3; i++) {
                v1[i] = _vertex1[i];
                v2[i] = _vertex2[i];
                v3[i] = _vertex3[i];
                n[i] = _normal[i];
            }
            return Face<T>(t3d*v1,t3d*v2,t3d*v3,t3d.R()*n);
        }


    };
}} // end namespaces

#endif // end include guard
