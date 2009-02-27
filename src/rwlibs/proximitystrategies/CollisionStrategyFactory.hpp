/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RWLIBS_PROXIMITYSTRATEGIES_COLLISIONSTRATEGYFACTORY_HPP
#define RWLIBS_PROXIMITYSTRATEGIES_COLLISIONSTRATEGYFACTORY_HPP

#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {

    class CollisionStrategyFactory
    {
    public:
    	/**
    	 * @brief function to create a default available collision strategy
    	 * @return NULL if no collisionstrategies are available else a Ptr to a
    	 * collision strategy
    	 */
    	static rw::common::Ptr<rw::proximity::CollisionStrategy> makeDefaultCollisionStrategy();

    };

}} // end namespaces

#endif // end include guard
