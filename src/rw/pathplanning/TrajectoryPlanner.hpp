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

#ifndef rw_pathplanning_TrajectoryPlanner_HPP
#define rw_pathplanning_TrajectoryPlanner_HPP

/**
 * @file TrajectoryPlanner.hpp
 */

#include "Path.hpp"
#include <rw/common/PropertyMap.hpp>
#include <list>

#include <rw/interpolator/Pose6dStraightSegment.hpp>


namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
     * @brief DEPRECATED An abstract ToolTrajectoryPlanner class
     *
     * The TrajectoryPlanner Interface is deprecated.
     *
     * Implement concrete TrajectoryPlanners by subclassing this class
     */
    class TrajectoryPlanner{
    protected:
		 typedef math::Q Q;

    public:
        /**
         * @brief Destroys object
         */
        virtual ~TrajectoryPlanner();

        /**
		 * @brief Solve inverse kinematics along a trajectory interpolator, stores result in path
		 * @return true if succes, false if error
         */
		virtual bool solve(const Q& qInit,
			   const rw::interpolator::Pose6dStraightSegment& interpolator,
               Path& path) = 0;

        /**
         * @brief Returns the PropertyMap for the Planner
         * @return Reference to PropertyMap
         */
        virtual common::PropertyMap& getProperties();

        /**
         * @brief Returns the PropertyMap for the planner
         * @return Reference to PropertyMap
         */
        virtual const common::PropertyMap& getProperties() const;

    protected:
        TrajectoryPlanner()
        {}

        //! PropertyMap for planner
        common::PropertyMap _properties;

    private:
        TrajectoryPlanner(const TrajectoryPlanner&);
        TrajectoryPlanner& operator=(const TrajectoryPlanner&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
