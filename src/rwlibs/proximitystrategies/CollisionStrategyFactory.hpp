
#ifndef COLLISIONSTRATEGYFACTORY_HPP_
#define COLLISIONSTRATEGYFACTORY_HPP_

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
