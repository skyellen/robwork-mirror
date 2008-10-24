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

#ifndef RWLIBS_LUA_PATHPLANNERFACTORY_HPP
#define RWLIBS_LUA_PATHPLANNERFACTORY_HPP

/**
   @file PathPlannerFactory.hpp
*/

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QToTPlanner.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/models/Device.hpp>

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    /**
       @brief Factory for PathPlanner objects for Lua programs.
    */
    class PathPlannerFactory {
    public:

        /**
           @brief A tuple of common types of path planners.
        */
        struct Planner
        {
            /**
               @brief Constructor
            */
            Planner(
                rw::pathplanning::QToQPlannerPtr toQ,
                rw::pathplanning::QToTPlannerPtr toT)
                :
                toQ(toQ),
                toT(toT)
            {}

            rw::pathplanning::QToQPlannerPtr toQ;
            rw::pathplanning::QToTPlannerPtr toT;
        };

        /**
           @brief A path planner for a device, state and path planning
           constraint.
        */
        Planner make(
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            const rw::pathplanning::PlannerConstraint& constraint)
        { return doMake(device, state, constraint); }

        /**
           @brief Destructor
        */
        virtual ~PathPlannerFactory() {}

    protected:
        /**
           @brief Subclass implementation of the make() method.
        */
        virtual
        Planner doMake(
            rw::models::DevicePtr device,
            const rw::kinematics::State& state,
            const rw::pathplanning::PlannerConstraint& constraint) = 0;

    private:
        PathPlannerFactory(const PathPlannerFactory&);
        PathPlannerFactory& operator=(const PathPlannerFactory&);

    protected:
        /**
           @brief Constructor
        */
        PathPlannerFactory() {}
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
