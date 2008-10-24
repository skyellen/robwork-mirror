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

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {

    class CollisionStrategyFactory
    {
    public:
#ifndef RW_REMOVE_DEPRECATED
/** DEPRECATED */
/*
  This conversion does not in any way depend on rw_proximitystrategies, and has therefore
  been moved to CollisionStrategy::make().
*/
        /**
         * @brief wraps a CollisionToleranceStrategy such that it
         * can be used as a CollisionStrategy
         */
        static rw::proximity::CollisionStrategy*
        NewCollisionStrategy(
            rw::proximity::CollisionToleranceStrategy* strategy,
            double tolerance);

#endif /* RW_REMOVE_DEPRECATED */
    };

}} // end namespaces

#endif // end include guard
