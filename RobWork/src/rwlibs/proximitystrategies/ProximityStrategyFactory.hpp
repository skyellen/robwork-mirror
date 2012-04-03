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


#ifndef RWLIBS_PROXIMITYSTRATEGIES_ProximityStrategyFactory_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_ProximityStrategyFactory_HPP

#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>
#include <rw/proximity/DistanceStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {

	/**
	 * @brief Factory class that enables constructing collision strategies
	 */
    class ProximityStrategyFactory
    {
    public:
        static std::vector<std::string> getCollisionStrategyIDs();

        /**
    	 * @brief function to create a default available collision strategy
    	 * @return NULL if no collisionstrategies are available else a Ptr to a
    	 * collision strategy
    	 */
		static rw::proximity::CollisionStrategy::Ptr makeDefaultCollisionStrategy();

		static rw::proximity::CollisionStrategy::Ptr makeCollisionStrategy(const std::string& id);

        static std::vector<std::string> getDistanceStrategyIDs();

		static rw::proximity::DistanceStrategy::Ptr makeDefaultDistanceStrategy();

		static rw::proximity::DistanceStrategy::Ptr makeDistanceStrategy(const std::string& id);


    };

}} // end namespaces

#endif // end include guard
