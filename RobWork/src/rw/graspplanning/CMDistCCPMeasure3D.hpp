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

#ifndef RW_GRASPPLANNING_CMDISTCCPMEASURE_HPP_
#define RW_GRASPPLANNING_CMDISTCCPMEASURE_HPP_

#include "GraspQualityMeasure3D.hpp"

#include <rw/math/Vector3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief computes the quality as a function of the distance between
 * the center of mass (COM) and the Center of the Contact Polygon (CCP)
 *
 * See ROA 3.2.3
 */
class CMDistCCPMeasure3D : public GraspQualityMeasure3D {
public:
    /**
     * @brief constructor
     * @param CM [in] The center of mass of the object described in the grasp frame.
     * @param maxDist [in] The max dist between CM and CCP that is allowed.
     * @return
     */
    CMDistCCPMeasure3D(const rw::math::Vector3D<>& CM, double maxDist):
        _CM(CM),_maxDist(maxDist)
    {}

    /**
     * @brief destructor
     */
    virtual ~CMDistCCPMeasure3D(){};

    /**
     * @brief computes the quality of the grasp such that the quality
     * is in the interval [0;1] with 1 being the highest quality.
     */
    virtual double quality(const Grasp3D& grasp) const;

private:
    rw::math::Vector3D<> _CM;
    double _maxDist;
};
}
}

#endif /*QUALITYMEASURE_HPP_*/
