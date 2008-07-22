/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rw_proximity_CollisionStrategy_HPP
#define rw_proximity_CollisionStrategy_HPP

/**
 * @file CollisionStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/geometry/Face.hpp>
#include <rw/common/Ptr.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    class CollisionStrategy;

    //! A pointer to a CollisionStrategy.
    typedef rw::common::Ptr<CollisionStrategy> CollisionStrategyPtr;

    /**
     * @brief The CDStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class CollisionStrategy : public virtual ProximityStrategy {
    public:
        /**
         * @brief Destroys object
         */
        virtual ~CollisionStrategy();

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in collision
         *
         * @param a [in] @f$ \mathcal{F}_a @f$
         *
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         *
         * @param b [in] @f$ \mathcal{F}_b @f$
         *
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         *
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb) = 0;

        /**
         * @brief Sets if the strategy should detect all colliding points or
         * only the first colliding point.
         *
         * @param b [in] Set to true if the strategy should return on first
         * detected colliding point.
         */
        virtual void setFirstContact(bool b) = 0;
        
    private:
        CollisionStrategy(const CollisionStrategy&);
        CollisionStrategy& operator=(const CollisionStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        CollisionStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
