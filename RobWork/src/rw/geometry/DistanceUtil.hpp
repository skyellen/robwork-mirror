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

#ifndef RW_GEOMETRY_DISTANCEUTIL_HPP_
#define RW_GEOMETRY_DISTANCEUTIL_HPP_

#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

    /**
     * @brief a class for performing distance calculations between different
     * geometric primitives
     */
    class DistanceUtil {
    public:
        /**
         * @brief computes the squared euclidean distance between line segments (line(p1,p2),line(q1,q2))
         * @param p1 [in] start point on line segment 1
         * @param p2 [in] end point on line segment 1
         * @param q1 [in] start point on line segment 2
         * @param q2 [in] end point on line segment 2
         * @return distance between line segments
         */
        static double distanceLineLineSqr(const rw::math::Vector3D<>& p1,
                                          const rw::math::Vector3D<>& p2,
                                          const rw::math::Vector3D<>& q1,
                                          const rw::math::Vector3D<>& q2);

        /**
         * @brief computes the euclidean distance between line segments (line(p1,p2),line(q1,q2))
         * @param p1 [in] start point on line segment 1
         * @param p2 [in] end point on line segment 1
         * @param q1 [in] start point on line segment 2
         * @param q2 [in] end point on line segment 2
         * @return distance between line segments
         */
        static double distanceLineLine(const rw::math::Vector3D<>& p1,
                                       const rw::math::Vector3D<>& p2,
                                       const rw::math::Vector3D<>& q1,
                                       const rw::math::Vector3D<>& q2);



    };

}
}

#endif /* DISTANCEUTIL_HPP_ */
