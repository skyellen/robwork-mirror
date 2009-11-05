/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


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
