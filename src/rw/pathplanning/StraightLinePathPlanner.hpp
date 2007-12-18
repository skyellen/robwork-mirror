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

#ifndef rw_pathplanning_StraightLinePathPlanner_HPP
#define rw_pathplanning_StraightLinePathPlanner_HPP

/**
 * @file StraightLinePathPlanner.hpp
 */

#include "PathPlanner.hpp"
#include "PlannerUtil.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
     * @brief A simple straight line path planner
     *
     * This is a simple straight line path planner that tries to make
     * a simple straight line path in joint space.
     *
     * Obviously this planner is NOT probablistic correct. This planner, however
     * is suitable as a local planner for more sophisticated path planners
     */
    class StraightLinePathPlanner : public PathPlanner
    {
    public:
        /**
         * @brief Creates object
         * @param device [in] the device
         * @param state [in] state of the workcell
         * @param detector [in] the collision detector to use
         * @param resolution [in] the resolution to use for collision checking
         */
        StraightLinePathPlanner(models::Device* device,
                                const kinematics::State& state,
                                proximity::CollisionDetector* detector,
                                double resolution);

        /**
         * @copydoc PathPlanner::query
         */
        bool query(const Q& qInit, const Q& qGoal, Path& path, double timeS = 60.0);

        /**
         * @brief Returns number of collision checks performed, usefull for statistics
         * @return number of collision checks since clear() was called
         */
        unsigned int nrCollisionChecks() const{
            return _collisionChecks;
        }

        /**
         * @brief Clears the number of collision checks counter
         */
        void clear(){
            _collisionChecks = 0;
        }

    private:
        bool interpolateMethod(const Q& start, const Q& end) const;
        PlannerUtil utils;
        models::Device* _device;
        double _resolution;
        mutable unsigned int _collisionChecks;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
