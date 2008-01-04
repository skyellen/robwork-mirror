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

#ifndef rwlibs_lua_PathPlannerFactory_HPP
#define rwlibs_lua_PathPlannerFactory_HPP

/**
 * @file PathPlannerFactory.hpp
 */

#include <memory>

namespace rw { namespace models { class WorkCell; class Device; }}
namespace rw { namespace kinematics { class Frame; }}

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    class PathPlanner;

    /**
       @brief Factory for PathPlanner objects for Lua programs.
    */
    class PathPlannerFactory {
    public:
        /**
           @brief A path planner for a workcell, device, tool frame and state.

           Ownership of \b workcell, \b device, and \b tcp is not taken.
        */
        virtual std::auto_ptr<PathPlanner> make(
            rw::models::WorkCell* workcell,
            rw::models::Device* device,
            rw::kinematics::Frame* tcp,
            const rw::kinematics::State& state) = 0;

        /**
           @brief Destructor
         */
        virtual ~PathPlannerFactory() {}

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
