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
 * @file PathPlanner.hpp
 */

#include "Path.hpp"
#include <rw/common/PropertyMap.hpp>
#include <list>

namespace rw { namespace models {
    class Device;
    class WorkCell;
}} // end namespaces

namespace rw { namespace proximity {
    class CollisionStrategy;
    class CollisionSetup;
}} // end namespaces

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /*@{*/

    /**
     * @brief DEPRECATED An abstract PathPlanner class
     *
     * The PathPlanner Interface is deprecated.
     *
     * Implement concrete PathPlanners by subclassing this class
     */
    class PathPlanner{

    public:
        /**
         * @brief Destroys object
         */
        virtual ~PathPlanner();

        /*
         * brief Initializes pathplanner
         *
         * param device [in] a device in the workcell that is to be used
         * for pathplanning
         *
         * pre device must be part of the workcell
         */
    //  virtual void initialize(models::Device* device);

        /**
         * @brief Perform path planning from @f$ \mathbf{q}_{init} @f$ to
         * @f$ \mathbf{q}_{goal} @f$
         *
         * @param qInit [in] @f$ \mathbf{q}_{init} @f$ initial position
         *
         * @param qGoal [in] @f$ \mathbf{q}_{goal} @f$ desired position
         *
         * @param path [out] a collision free path between @f$
         * \mathbf{q}_{init} @f$ and @f$ \mathbf{q}_{goal} @f$
         *
         * @param timeS [in] the amount of time to use inside the
         * pathplanner specified in seconds.
         *
         * @return true if a path between @f$ \mathbf{q}_{init} @f$ and @f$
         * \mathbf{q}_{goal} @f$ was found, false otherwise
         *
         * @pre the object must be initialized.
         *
         * Specific path planners must implement this method.
         */
        virtual bool query(const rw::math::Q& qInit,
                           const rw::math::Q& qGoal,
                           Path& path,
                           double timeS) = 0;
        /*
      The default parameter of timeS = 60 was killed from the query method.
      We shouldn't have default parameters on super classes, because it
      really is confusing that the value for the parameter may depend on the
      subtype. The call planner.query(init, goal, path) depends on the
      _static_ type of planner and not just its runtime type. Probably timeS
      shouldn't be a parameter at all. Instead you should have a Query
      object of sorts, or you should use the property map.
        */

        /**
         * @brief Returns the PropertyMap for the Planner
         * @return Reference to PropertyMap
         */
        virtual common::PropertyMap& getProperties();

        /**
         * @brief Returns the PropertyMap for the planner
         * @return Reference to PropertyMap
         */
        virtual const common::PropertyMap& getProperties() const;

        /**
         * @brief Sets whether to test the start configuration for collision.
         *
         * @param test [in] True to test
         */
        virtual void setTestQStart(bool test);

        /**
         * @brief Sets whether to test the start configuration for collision
         *
         * @return True if the start configuration should be tested
         */
        virtual bool testQStart() const;

        /**
         * @brief Sets whether to test the goal configuration for collision
         *
         * Default value is true when StraightLinePathPlanner is constructed
         *
         * @param test [in] True to test
         */
        void setTestQGoal(bool test);

        /**
         * @brief Sets whether to test the goal configuration for collision
         *
         * @return True if the goal configuration should be tested
         */
        virtual bool testQGoal() const;

    protected:
        PathPlanner()
        {
            _testQStart = true;
            _testQGoal = true;
        }

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
