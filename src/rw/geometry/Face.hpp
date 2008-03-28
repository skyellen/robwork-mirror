/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_geometry_Face_HPP
#define rw_geometry_Face_HPP

/**
 * @file Face.hpp
 */

#include <rw/math/Vector3D.hpp>

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

        //@brief default constructor
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

    };
}} // end namespaces

#endif // end include guard
