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

#ifndef RW_GRASPPLANNING_CONTACTDISTTHRESFILTER_HPP_
#define RW_GRASPPLANNING_CONTACTDISTTHRESFILTER_HPP_

#include "Grasp3D.hpp"
#include "GraspValidateFilter.hpp"

#include <rw/sensor/Contact3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief tests if contact points in a grasp is too close to each other in respect to the
 * approach angle.
 *
 * Two points that are very close is not allowed unless they are approached from opposite
 * directions.
 *
 */
class ContactDistThresFilter: public GraspValidateFilter {
public:

    /**
     *
     * @param minDist
     * @param maxDist
     * @param allowCloseWhenOpposite
     * @return
     */
    ContactDistThresFilter(double minDist, double maxDist, bool allowCloseWhenOpposite = true):
        _minDist(minDist), _maxDist(maxDist),_allowCloseWhenOpposite(allowCloseWhenOpposite)
    {};

    /**
     * @brief destructor
     */
    virtual ~ContactDistThresFilter(){};

    /**
     * @copydoc GraspValidateFilter::isValid
     */
    bool isValid(const Grasp3D& grasp);

    /**
     * @brief tests if the contact pair is valid according to this filter
     * @param c1
     * @param c2
     * @return
     */
    bool isContactPairValid(const rw::sensor::Contact3D& c1, const rw::sensor::Contact3D& c2);

private:
    double _minDist;
    double _maxDist;
    bool _allowCloseWhenOpposite;
};
}
}
#endif /* PLANECLEARANCEFILTER_HPP_ */
