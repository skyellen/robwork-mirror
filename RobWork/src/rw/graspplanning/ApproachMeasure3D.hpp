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


#ifndef RW_GRASPPLANNING_APPROACHMEASURE_HPP_
#define RW_GRASPPLANNING_APPROACHMEASURE_HPP_

#include "GraspQualityMeasure3D.hpp"

namespace rw {
namespace graspplanning {
    /**
     * @brief computes the quality as a function of the angle between
     * the approach angle and some planar surface.
     */
    class ApproachMeasure3D : public GraspQualityMeasure3D {
    public:
        /**
         * @brief constructor
         * @param maxAngle [in] the maximum allowed angle between approach and
         * surface normal. Grasps violating this will recieve high penalty on
         * quality.
         * @return
         */
        ApproachMeasure3D(double maxAngle):
            _maxAngle(maxAngle)
        {}

        /**
         * @brief destructor
         */
        virtual ~ApproachMeasure3D(){};

        /**
         * @brief computes the quality of the grasp such that the quality
         * is in the interval [0;1] with 1 being the highest quality.
         */
        virtual double quality(const Grasp3D& grasp) const;

    private:

        double _maxAngle;
    };

}
}

#endif /*QUALITYMEASURE_HPP_*/
