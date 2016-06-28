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


#ifndef RW_PATHPLANNING_PATHPLANNER_HPP
#define RW_PATHPLANNING_PATHPLANNER_HPP

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
       @brief Path planner interface.

       PathPlanner<From, To, Path> plans a path in the configuration space \b
       From from a start configuration of type \b From to a goal destination
       specified by a parameter of type \b To. The path is of type \b Path.
    */
	template <class From, class To, class Path = rw::trajectory::Path<From> >
    class PathPlanner
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<PathPlanner> Ptr;
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
        bool query(const From& from,
                   To& to,
                   Path& path,
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
            const From& from,
            To& to,
            Path& path,
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
            const From& from,
            To& to,
            Path& path)
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
            const From& from,
            To& to,
            Path& path,
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
