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

#ifndef RW_SENSOR_CONTACT3D_HPP_
#define RW_SENSOR_CONTACT3D_HPP_

#include <rw/math/Vector3D.hpp>

namespace rw {
namespace sensor {

/**
 * @brief data structure for describing a contact in 3D
 */
class Contact3D
{
public:
    //! @brief constructor
    Contact3D() :
        normalForce(0.0),
        _faceIdx(0),
        _faceIdx2(0),
        curvature(0.0),
        mu(0.6)
    {
    }

    /**
     * @brief constructor
     * @param tp [in] point contact
     * @param tn [in] contact normal
     * @param normalf [in] normal force in the contact
     */
    Contact3D(rw::math::Vector3D<> tp, rw::math::Vector3D<> tn, double normalf) :
        p(tp), n(tn), f(n * normalf), normalForce(normalf),
        _faceIdx(0),
        _faceIdx2(0),
        curvature(0.0),
        mu(0.6)
    {
    }

    /**
     * @brief constructor
     * @param tp [in] point contact
     * @param tn [in] contact normal
     * @param tf [in] force in the contact
     */
    Contact3D(rw::math::Vector3D<> tp, rw::math::Vector3D<> tn, rw::math::Vector3D<> tf) :
        p(tp), n(tn), f(tf),
        _faceIdx(0),
        _faceIdx2(0),
        curvature(0.0),
        mu(0.6)
    {
        normalForce = dot(f, n);
    }


    rw::math::Vector3D<> p; //! Contact position
    rw::math::Vector3D<> n; //! Surface contact normal
    rw::math::Vector3D<> f; //! the actual force
    double normalForce; //!  normal force

    //! index to the geometric primitive on which the contact is located
    unsigned int _faceIdx, _faceIdx2;

    // hmm, dunno about 3d curvature
    double curvature; //! surface curvature
    double mu; //! coulomb friction coefficient
};

}
}

#endif /*RW_SENSOR_CONTACT3D_HPP*/

