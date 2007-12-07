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

#ifndef rwlibs_lua_PathPlanner_HPP
#define rwlibs_lua_PathPlanner_HPP

/**
 * @file PathPlanner.hpp
 */

#include <vector>

namespace rw { namespace models { class DeviceModel; }}
namespace rw { namespace kinematics { class State; }}
namespace rw { namespace math { class Q; }}

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    /**
       @brief Simple PathPlanner for Lua programs.
    */
    class PathPlanner {
    public:
        typedef std::vector<rw::kinematics::State> Path;

        /**
           @brief Plan a path for \b device from \b from to \b to for a shared
           state of \b state.
        */
        virtual Path query(
            rw::models::DeviceModel& device,
            const rw::kinematics::State& state,
            const rw::math::Q& from,
            const rw::math::Q& to) = 0;

        /**
           @brief Destructor
         */
        virtual ~PathPlanner() {}

    private:
        PathPlanner(const PathPlanner&);
        PathPlanner& operator=(const PathPlanner&);

    protected:
        /**
           @brief Constructor
         */
        PathPlanner() {}
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
