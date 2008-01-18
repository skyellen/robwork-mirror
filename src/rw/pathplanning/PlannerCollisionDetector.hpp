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

#ifndef rw_pathplanning_PlannerCollisionDetector_HPP
#define rw_pathplanning_PlannerCollisionDetector_HPP

/**
 * @file PlannerCollisionDetector.hpp
 */

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace models { class Device; }}
namespace rw { namespace proximity { class CollisionDetector; }}

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /** @brief Collision checking for configuration space path planners.
     *
     * PlannerCollisionDetector is a simply utility class that provides
     * collision checking for configurations Q for a device. The assumption is
     * that the Q values are the only state values of the workcell that are
     * being changed.
     */
    class PlannerCollisionDetector
    {
    public:
        /**
         * @brief Constructor
         *
         * @param detector [in] The collision checker.
         *
         * @param device [in] The device to which the Q values belong.
         *
         * @param state [in] The state relative to which the collision checking
         * is done.
         */
        PlannerCollisionDetector(
            proximity::CollisionDetector* detector,
            models::Device* device,
            const kinematics::State& state);

        /**
         * @brief True iff the workcell state collides for a configuration \b q
         * for the device.
         */
        bool inCollision(const math::Q& q) const;

        /**
         * @brief The device to which the Q values belong.
         */
        models::Device& getDevice() const { return *_device; }

    private:
        proximity::CollisionDetector* _detector;
        models::Device* _device;
        kinematics::State _state;

    private:
        PlannerCollisionDetector(const PlannerCollisionDetector&);
        PlannerCollisionDetector& operator=(const PlannerCollisionDetector&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
