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

#ifndef rw_proximity_CollisionToleranceStrategy_HPP
#define rw_proximity_CollisionToleranceStrategy_HPP

/**
 * @file CollisionStrategy.hpp
 */

#include <string>

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/geometry/Face.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/

    /**
     * @brief The CDStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class CollisionToleranceStrategy: public virtual ProximityStrategy {
    public:
        /**
         * @brief Destroys object
         */
        virtual ~CollisionToleranceStrategy();

        /**
         * @brief Checks to see if two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$ are in CollisionTolerance
         *
         * @param a [in] @f$ \mathcal{F}_a @f$
         *
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         *
         * @param b [in] @f$ \mathcal{F}_b @f$
         *
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         * 
         * @param tolerance [in] frames with a distance in between them
         * that is less than tolerance are in collision 
         * 
         * @return true if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * colliding, false otherwise.
         */
        virtual bool inCollision(
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame *b,
            const math::Transform3D<>& wTb,
            double tolerance) = 0;

    private:
        CollisionToleranceStrategy(const CollisionToleranceStrategy&);
        CollisionToleranceStrategy& operator=(const CollisionToleranceStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        CollisionToleranceStrategy();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
