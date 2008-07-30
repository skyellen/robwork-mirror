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

#ifndef rw_pathplanning_PathPlanner_HPP
#define rw_pathplanning_PathPlanner_HPP

/**
   @file PathPlanner.hpp
*/

#include <rw/trajectory/Path.hpp>
#include "StopCriteria.hpp"
#include <rw/common/PropertyMap.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Destination planner interface.

       PathPlanner<Destination> plans a path in the configuration space
       from a start configuration to a goal destination given by a parameter of
       type Destination.
    */
    template <class Destination>
    class PathPlanner
    {
    public:
        /**
           @brief Destructor
        */
        virtual ~PathPlanner() {}

        /**
           @brief Plan a path from the configuration \b from to the destination
           \b to.

           @param from [in] start configuration for path.

           @param to [in] end destination of path.

           @param path [out] a collision free path connecting \b from to \b to.

           @param stop [in] Abort the planning when \b stop returns true.

           @return true if a path between from \b from to \b to was found and
           false otherwise.
        */
        bool query(
            const rw::math::Q& from,
            Destination& to,
            rw::trajectory::QPath& path,
            const StopCriteria& stop)
        {
            return doQuery(from, to, path, stop);
        }

        /**
           @brief Plan a path from the configuration \b from to the destination
           \b to.

           @param from [in] start configuration for path.

           @param to [in] end destination of path.

           @param path [out] a collision free path connecting \b from to \b to.

           @param time [in] Abort the planning after \b time seconds.

           @return true if a path between from \b from to \b to was found and
           false otherwise.
        */
        bool query(
            const rw::math::Q& from,
            Destination& to,
            rw::trajectory::QPath& path,
            double time)
        {
            return query(from, to, path, *StopCriteria::stopAfter(time));
        }

        /**
           @brief Plan a path from the configuration \b from to the destination
           \b to.

           The planner runs until it gives up (which may be never).

           @param from [in] start configuration for path.

           @param to [in] end destination of path.

           @param path [out] a collision free path connecting \b from to \b to.

           @return true if a path between from \b from to \b to was found and
           false otherwise.
        */
        bool query(
            const rw::math::Q& from,
            Destination& to,
            rw::trajectory::QPath& path)
        {
            return query(from, to, path, *StopCriteria::stopNever());
        }

        /**
           @brief Property map for the planner.
        */
        common::PropertyMap& getProperties() { return _properties; }

        /**
           @brief Property map for the planner.
        */
        const common::PropertyMap& getProperties() const { return _properties; }

    protected:
        /**
           @brief Default constructor provided for subclasses.
        */
        PathPlanner() {}

        /**
           @brief Subclass implementation of the query() method.
        */
        virtual bool doQuery(
            const rw::math::Q& from,
            Destination& to,
            rw::trajectory::QPath& path,
            const StopCriteria& stop) = 0;

    private:
        common::PropertyMap _properties;

    private:
        PathPlanner(const PathPlanner&);
        PathPlanner& operator=(const PathPlanner&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
