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


#ifndef RWLIBS_LUA_ROBWORH_HPP
#define RWLIBS_LUA_ROBWORH_HPP

/**
 * @file RobWork.hpp
 */

#include "PathPlannerFactory.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <rw/trajectory/Path.hpp>

struct lua_State;

namespace rw { namespace proximity { class CollisionDetector; class CollisionStrategy; }}
namespace rw { namespace kinematics { class State; }}
namespace rw { namespace models { class WorkCell; }}

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    class Output;

    /** @brief A Lua module for RobWork.
     */
    class RobWork
    {
    public:
        /**
           @brief Load the RobWork package into Lua.

           This function is a wrapper of a tolua_pkgname_open() call where
           pkgname is internally defined in this library.

           @return 0 if and and only if the loading succeeded.
        */
        static int open(lua_State* L);

        /**
           @brief Set the output collector of the RobWork package.

           The output collector is used by Lua commands such as rw.write().

           Ownership of \b output is not taken.
         */
        static void setOutput(lua_State* L, Output* output);

        /**
           @brief Set the state of the RobWork package.

           A \e copy of the state is retrievable by the Lua command
           rw.getState().

           Writes to the state via the Lua script is handled by \b listener.

           Ownership of \b state is not taken.
        */
        static void setState(
            lua_State* L,
            rw::kinematics::State* state);

        /**
           @brief Event handler for writes to the state of the RobWork package.

           The RobWork state was set by setState().
        */
        typedef
        boost::function<void(const rw::kinematics::State&)>
        StateChangedListener;

        /**
           @brief Event handler for writes to the path of the RobWork package.

           The RobWork state was set by setState().
        */
        typedef boost::function<void(const rw::trajectory::TimedStatePath&)>
        PathChangedListener;

        /**
           @brief Assign an event handler for writes to the RobWork state.
         */
        static void setStateChangedListener(
            const StateChangedListener& listener);

        /*
           @brief The event handler for writes to the RobWork state.
        */
        static const StateChangedListener& getStateChangedListener();

        /**
           @brief Assign an event handler for writes to the RobWork path.
         */
        static void setPathChangedListener(
            const PathChangedListener& listener);

        /*
           @brief The event handler for writes to the RobWork path.
        */
        static const PathChangedListener& getPathChangedListener();

        /**
           @brief Set the workcell of the RobWork package.

           A reference to the workcell is retrievable by the Lua command
           rw.getState().

           Ownership of \b workcell is not taken.
        */
        static void setWorkCell(lua_State* L, rw::models::WorkCell* workcell);

        /**
           @brief Set the collision detector of the RobWork package.

           A reference to the workcell is retrievable by the Lua command
           rw.getCollisionDetector().

           Ownership of \b detector is not taken.
        */
        static void setCollisionDetector(
            lua_State* L, rw::proximity::CollisionDetector* detector);

        /**
           @brief Set the collision strategy of the RobWork package.

           A reference to the workcell is retrievable by the Lua command
           rw.getCollisionStrategy().

           Ownership of \b strategy is not taken.
        */
        static void setCollisionStrategy(
            lua_State* L, rw::proximity::CollisionStrategy* strategy);

        /**
           @brief Set the path-planner factory of the RobWork package.

           The planners of the factory can be retrieved with the Lua command
           rw.getPathPlanner(device, options).

           Ownership of \b factory is not taken.
        */
        static
        void setPathPlannerFactory(lua_State* L, PathPlannerFactory* factory);

    private:
        RobWork();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
