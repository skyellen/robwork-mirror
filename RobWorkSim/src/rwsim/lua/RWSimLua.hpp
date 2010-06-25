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


#ifndef RWSimLua_HPP
#define RWSimLua_HPP

/**
 * @file RWSimLua.hpp
 */

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <rw/trajectory/Path.hpp>

struct lua_State;

namespace rwsim {
namespace lua {


    /** @addtogroup lua */
    /*@{*/

    class Output;

    /** @brief A Lua module for RobWork.
     */
    class RWSimLua
    {
    public:
        /**
           @brief Load the RWSimLua package into Lua.

           This function is a wrapper of a tolua_pkgname_open() call where
           pkgname is internally defined in this library.

           @return 0 if and and only if the loading succeeded.
        */
        static int open(lua_State* L);

        /**
           @brief Set the output collector of the RWSimLua package.

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
           @brief Set the dynamic workcell of the RWSimLua package.

           A reference to the workcell is retrievable by the Lua command rw.getState().

           Ownership of \b workcell is not taken.
        */
        static void setDynamicWorkCell(lua_State* L, dynamics::DynamicWorkCell* workcell);

        /**
           @brief Get the dynamic workcell of the RWSimLua package.
        */
        static dynamics::DynamicWorkCell* getDynamicWorkCell(lua_State* L);

    private:
        RWSimLua();
    };

    /*@}*/
}
} // end namespaces
#endif // end include guard
