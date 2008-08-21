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

           A reference to the workcell is retrievable by the Lua command rw.getState().

           Ownership of \b workcell is not taken.
        */
        static void setWorkCell(lua_State* L, rw::models::WorkCell* workcell);

        /**
           @brief Set the collision detector of the RobWork package.

           A reference to the workcell is retrievable by the Lua command rw.getCollisionDetector().

           Ownership of \b detector is not taken.
        */
        static void setCollisionDetector(
            lua_State* L, rw::proximity::CollisionDetector* detector);

        /**
           @brief Set the collision strategy of the RobWork package.

           A reference to the workcell is retrievable by the Lua command rw.getCollisionStrategy().

           Ownership of \b strategy is not taken.
        */
        static void setCollisionStrategy(
            lua_State* L, rw::proximity::CollisionStrategy* strategy);

    private:
        RobWork();
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
