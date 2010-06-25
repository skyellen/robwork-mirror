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


#ifndef RW_COLLISION_PROXIMITY_HPP
#define RW_COLLISION_PROXIMITY_HPP

#include "CollisionStrategy.hpp"
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/geometry/Geometry.hpp>

#include <vector>
#include <set>

namespace rw { namespace proximity {

	/** @addtogroup proximity */
	/*@{*/

	/**
	 * @brief A pair of frame names
	 */
	typedef std::pair<std::string, std::string> ProximityPair;

	/**
	 * @brief A list of pairs for with ProximityPairs
	 */
	typedef std::vector<ProximityPair> ProximityPairList;

	class CollisionSetup;

    /**
       @brief Utility functions for the rw::proximity module.
    */
    class Proximity {
    public:

        /**
           @brief The collision setup of the workcell.

           If no collision setup is stored in the workcell, then the empty
           collision setup is returned.

           @param workcell [in] Workcell containing a collision setup.
        */
        static
        CollisionSetup getCollisionSetup(const rw::models::WorkCell& workcell);

        static
        std::vector<rw::geometry::GeometryPtr> getGeometry(const rw::kinematics::Frame* frame);
    private:
        Proximity();
        Proximity(const Proximity&);
        Proximity& operator=(const Proximity&);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
