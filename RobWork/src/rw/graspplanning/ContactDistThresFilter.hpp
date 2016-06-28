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

#include "GraspValidateFilter.hpp"

namespace rw { namespace sensor { class Contact3D; } }

namespace rw {
namespace graspplanning {

/**
 * @brief tests if contact points in a grasp is too close or too far from each other.
 *
 * Two points that are very close is not allowed unless they are approached from opposite
 * directions.
 *
 */
class ContactDistThresFilter: public GraspValidateFilter {
public:

    /**
     * @brief constructor
     * @param minDist [in] minimum allowed distance between contact points
     * @param maxDist [in] maximum allowed distance between contact points
     * @param allowCloseWhenOpposite [in] if true small distances are allowed when contact normals are
     * in opposite directions
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
     * @param c1 [in] 3d contact
     * @param c2 [in] 3d contact
     * @return true if contact pair is within filter criterias, false otherwise
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
