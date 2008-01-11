/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef rwlibs_lua_RobWork_HPP
#define rwlibs_lua_RobWork_HPP

/**
 * @file RobWork.hpp
 */

#include <boost/shared_ptr.hpp>

struct lua_State;

namespace rw { namespace kinematics { class State; }}
namespace rw { namespace models { class WorkCell; }}

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    class Output;
    class PathPlannerFactory;

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

           Ownership of \b state is not taken.
        */
        static void setState(lua_State* L, rw::kinematics::State* state);

        /**
           @brief Set the workcell of the RobWork package.

           A reference to the workcell is retrievable by the Lua command rw.getState().

           Ownership of \b workcell is not taken.
        */
        static void setWorkCell(lua_State* L, rw::models::WorkCell* workcell);

        /**
           @brief Set the path-planner factory of the RobWork package.

           A reference to a constructed planner is retrievable by the Lua
           command rw.getPathPlanner(workcell, device, tcp, state).

           Ownership of \b factory is not taken.
        */
        static void setPathPlannerFactory(lua_State* L, PathPlannerFactory* factory);

    private:
        RobWork();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
