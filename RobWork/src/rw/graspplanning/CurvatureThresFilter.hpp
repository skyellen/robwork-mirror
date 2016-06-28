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

#ifndef RW_GRASPPLANNING_CURVATURETHRESFILTER_HPP_
#define RW_GRASPPLANNING_CURVATURETHRESFILTER_HPP_

#include "GraspValidateFilter.hpp"
#include "ContactValidateFilter.hpp"

namespace rw {
namespace graspplanning {

/**
 * @brief tests if a grasp is valid in respect to the curvature of the object
 * surface in and around the contact points.
 *
 * This class requires that the face in which the contact point is extracted is
 * registered in the Contact3D data.
 *
 */
class CurvatureThresFilter: public GraspValidateFilter, public ContactValidateFilter {
public:

    /**
     * @brief constructor
     * @param minCurvature [in] minimum curvature
     * @param maxCurvature [in] maximum curvature
     * @return
     */
    CurvatureThresFilter(double minCurvature, double maxCurvature):
        _minCurvature(minCurvature), _maxCurvature(maxCurvature)
    {};

    //! destructor
    virtual ~CurvatureThresFilter(){};

    //! @copydoc GraspValidateFilter::isValid
    bool isValid(const Grasp3D& grasp);

    //! @copydoc ContactValidateFilter::isValid
    bool isValid(const rw::sensor::Contact3D& contact);

private:
    double _minCurvature;
    double _maxCurvature;
};

}
}

#endif /* PLANECLEARANCEFILTER_HPP_ */
