#ifndef COLLISIONSTRATEGYFACTORY_HPP_
#define COLLISIONSTRATEGYFACTORY_HPP_

#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/proximity/CollisionToleranceStrategy.hpp>

namespace rwlibs { namespace proximitystrategies {

    class CollisionStrategyFactory
    {
    public:
        /**
         * @brief wraps a CollisionToleranceStrategy such that it
         * can be used as a CollisionStrategy
         */
        static rw::proximity::CollisionStrategy*
        NewCollisionStrategy(
            rw::proximity::CollisionToleranceStrategy* strategy,
            double tolerance);
    };

}} // end namespaces

#endif // end include guard
