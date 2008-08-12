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
   @file rwlibs/lua/PathPlanner.hpp
*/

#include <vector>
#include <rw/math/Transform3D.hpp>
#include <rw/common/Ptr.hpp>

namespace rw { namespace models { class Device; }}
namespace rw { namespace kinematics { class State; }}
namespace rw { namespace math { class Q; }}

namespace rwlibs { namespace lua {

    /** @addtogroup lua */
    /*@{*/

    class PathPlanner;

    //! A pointer to a PathPlanner.
    typedef rw::common::Ptr<PathPlanner> PathPlannerPtr;

    /**
       @brief Simple PathPlanner for Lua programs.
    */
    class PathPlanner {
    public:
        /**
         * @brief Definition of Path for lua
         */
        typedef std::vector<rw::kinematics::State> Path;

        /**
           @brief Plan a path from \b from to \b to.
        */
        virtual Path query(
            const rw::math::Q& from,
            const rw::math::Q& to) = 0;

        /**
           @brief Plan an approach path from \b from to \b to.
         */
        virtual Path query(
            const rw::math::Q& from,
            const rw::math::Transform3D<>& to) = 0;

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
