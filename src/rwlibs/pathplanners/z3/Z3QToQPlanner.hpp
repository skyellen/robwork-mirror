/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RWLIBS_PATHPLANNERS_Z3_Z3QTOQPLANNER_HPP
#define RWLIBS_PATHPLANNERS_Z3_Z3QTOQPLANNER_HPP

/**
   @file Z3QToQPlanner.hpp
*/

#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QSampler.hpp>

#include <vector>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief Z3 based point-to-point planner.
    */
    class Z3QToQPlanner : public rw::pathplanning::QToQPlanner
    {
    public:
        /**
           @brief Constructor

           @param sampler [in] Sampler of the configuration space.

           @param localPlanner [in] Local planner for connecting the configurations.

           @param nodeCnt [in] Number of supporting configurations to insert.
           If \b nodeCnt is negative, a default value is chosen.

           @param repeatCnt [in] Number of times to repeat the attempt. If \b
           repeatCnt is negative, the attempts are repeated forever (or until
           the stop criteria returns true).
        */
        Z3QToQPlanner(
            rw::pathplanning::QSamplerPtr sampler,
            rw::pathplanning::QToQPlannerPtr localPlanner,
            int nodeCnt,
            int repeatCnt);

    private:
        typedef rw::trajectory::QPath Path;

        bool doQuery(
            const rw::math::Q& start,
            const rw::math::Q& goal,
            Path& path,
            const rw::pathplanning::StopCriteria& stop);

    private:
        rw::pathplanning::QSamplerPtr _sampler;
        rw::pathplanning::QToQPlannerPtr _localPlanner;
        int _nodeCnt;
        int _repeatCnt;
    };

    /*\}*/
}} // end namespaces

#endif // end include guard
