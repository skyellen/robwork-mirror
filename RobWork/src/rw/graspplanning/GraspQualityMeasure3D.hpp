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

#ifndef RW_GRASPPLANNING_GRASPQUALITYMEASURE3D_HPP_
#define RW_GRASPPLANNING_GRASPQUALITYMEASURE3D_HPP_

#include <rw/common/Ptr.hpp>

namespace rw {
namespace graspplanning {

class Grasp3D;

/**
 * @brief an interface for methods evaluating the quality of a specific grasp
 */
class GraspQualityMeasure3D {
public:
    //! smart pointer type of this class
    typedef rw::common::Ptr<GraspQualityMeasure3D> Ptr;

    /**
     * @brief computes the quality of the grasp such that the quality
     * is in the interval [0;1] with 1 being the highest quality.
     */
    virtual double quality(const Grasp3D& grasp) const = 0;

};



}
}

#endif /*GRASPQUALITYMEASURE3D_HPP_*/
