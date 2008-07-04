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

#include "Path.hpp"
#include "StopCriteria.hpp"
#include <rw/common/PropertyMap.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
       @brief Path planner interface.

       A path planner plans a path connecting a pair of configurations.
    */
    class PathPlanner
    {
    public:
        /**
           @brief Destructor
        */
        virtual ~PathPlanner();

        /**
           @brief Plan a path from \b from to \b to.

           @param from [in] Start configuration for the path.

           @param to [in] End configuration for the path.

           @param path [out] A collision free path connecting \b from to \b to.

           @param stop [in] Abort the planning when \b stop returns true.

           @return True if a path between from \b from to \b to was found and
           false otherwise.
        */
        bool query(
            const rw::math::Q& from,
            const rw::math::Q& to,
            Path& path,
            const StopCriteria& stop);

        /**
           @brief Perform path planning from @f$ \mathbf{q}_{init} @f$ to
           @f$ \mathbf{q}_{goal} @f$.

           This method forwards to the general query() method, but tells the
           planner to stop after \b time seconds.
        */
        bool query(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            Path& path,
            double time);

        /**
           @brief Perform path planning from @f$ \mathbf{q}_{init} @f$ to
           @f$ \mathbf{q}_{goal} @f$.

           This method forwards to the general query() method, but and lets the
           planner run forever.
        */
        bool query(
            const rw::math::Q& qInit,
            const rw::math::Q& qGoal,
            Path& path);

        /**
           @brief Returns the PropertyMap for the Planner
           @return Reference to PropertyMap
        */
        virtual common::PropertyMap& getProperties();

        /**
           @brief The PropertyMap for the planner
           @return Reference to PropertyMap
        */
        virtual const common::PropertyMap& getProperties() const;

        /**
           @brief Sets whether to test the start configuration for collision.

           @param test [in] True to test
        */
        virtual void setTestQStart(bool test);

        /**
           @brief Sets whether to test the start configuration for collision

           @return True if the start configuration should be tested
        */
        virtual bool testQStart() const;

        /**
           @brief Sets whether to test the goal configuration for collision

           Default value is true when StraightLinePathPlanner is constructed

           @param test [in] True to test
        */
        void setTestQGoal(bool test);

        /**
           @brief Sets whether to test the goal configuration for collision

           @return True if the goal configuration should be tested
        */
        virtual bool testQGoal() const;

    protected:
        /**
           @brief Default constructor for use by subclasses.
        */
        PathPlanner();

        /**
           @brief Subclass implementation of the query() method.
        */
        virtual bool doQuery(
            const rw::math::Q& from,
            const rw::math::Q& to,
            Path& path,
            const StopCriteria& stop) = 0;

    private:
        //! PropertyMap for planner
        common::PropertyMap _properties;

        //! Specifies whether to test the start configuration
        bool _testQStart;

        //! Specifies whether to test the goal configuration
        bool _testQGoal;

    private:
        PathPlanner(const PathPlanner&);
        PathPlanner& operator=(const PathPlanner&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
