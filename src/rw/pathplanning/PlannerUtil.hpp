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

#ifndef rw_pathplanning_PlannerUtil_HPP
#define rw_pathplanning_PlannerUtil_HPP

/**
 * @file PlannerUtil.hpp
 */

#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    /**
     * @brief PlannerUtil provides various utilities useful in path planning
     */
    class PlannerUtil {
    public:

        /**
         * @brief Constructs PlannerUtil object
         *
         * The object does NOT take ownership of or copies the workcell
         * or collision detector. The caller must therefore make sure that
         * these stays valid while using the object
         *
         * @param device [in] the device
         * @param state [in] the default state to use
         * @param detector [in] the collision detector to use
         */
        PlannerUtil(
            rw::models::Device* device,
            const rw::kinematics::State& state,
            rw::proximity::CollisionDetector* detector);

        /**
         * @brief destructor
         */
        ~PlannerUtil();

        /**
         * @brief sets the default state
         * @param state [in] the state to use
         */
        void setState(rw::kinematics::State& state);

        /**
         * @brief Creates random joint configuration @f$ \mathbf{q}\in C @f$
         * @return a joint configuration
         * @pre the object must be initialized
         */
        rw::math::Q randomConfig() const;

        /**
         * @brief Checks to see if the robot is in collision in the given
         * joint configuration (@f$ \mathbf{q}\in C_{free} @f$)
         *
         * @param q [in] @f$ \mathbf{q} @f$ the configuration to check
         * @return true if the robot is in collision, false otherwise
         * @pre the object must be initialized
         */
        bool inCollision(const rw::math::Q& q) const;

        /**
         * @brief Normalizes configuration
         * @param q [in] @f$ \mathbf{q} @f$
         * @return @f$ \mathbf{qn} @f$
         * @pre the object must be initialized
         */
        rw::math::Q normalize(const rw::math::Q& q) const;

        /**
         * @brief UnNormalizes configuration
         * @param qn [in] @f$ \mathbf{qn} @f$
         * @return @f$ \mathbf{q} @f$
         * @pre the object must be initialized
         */
        rw::math::Q unNormalize(const rw::math::Q& qn) const;

    private:
        rw::models::Device* _device;
        rw::kinematics::State _state;
        rw::proximity::CollisionDetector* _collisionDetector;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
