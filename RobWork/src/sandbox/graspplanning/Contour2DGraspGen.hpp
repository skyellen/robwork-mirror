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

#ifndef RW_GRASPPLANNING_CONTOUR2DGRASPGEN_HPP_
#define RW_GRASPPLANNING_CONTOUR2DGRASPGEN_HPP_

#include "Contour2DInfoMap.hpp"
#include <rw/math/Vector2D.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics/State.hpp>
#include "QualityMeasure2D.hpp"
#include "Grasp2D.hpp"

namespace rw {
namespace graspplanning {

    /**
     * @brief generates N good contact points on the 2D contour
     */
    class Contour2DGraspGen {
    public:

        /**
         * @brief
         */
        virtual void reset( const rw::geometry::Contour2D& contour ) = 0;

        /**
         * @brief computes a list of the best grasps in the grasp candidate list.
         */
        virtual std::vector<Grasp2D>
            computeGrasps(const QualityMeasure2D& measure, double minQuality, const unsigned int nrOfGrasps) = 0;

        /**
         * @brief gets grasp candidate at idx
         */
        virtual Grasp2D getGrasp(int idx) = 0 ;
    };

}
}

#endif /*CG3Grasp2DGen_HPP_*/
