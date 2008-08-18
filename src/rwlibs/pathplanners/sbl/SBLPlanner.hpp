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

#ifndef rwlibs_pathplanners_sbl_SBLPlanner_HPP
#define rwlibs_pathplanners_sbl_SBLPlanner_HPP

/**
   @file SBLPlanner.hpp
*/

#include "SBLSetup.hpp"
#include <rw/pathplanning/QToTPlanner.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/pathplanning/QToQSamplerPlanner.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

    /**
       @brief SBL based planners.

       @relates QToQPlanner
       @relates QToTPlanner
       @relates QToQSamplerPlanner
    */
    class SBLPlanner
    {
    public:
        /**
           @brief An SBL based sampled region planner.

           @param setup [in] Setup for the planner.
        */
        static std::auto_ptr<rw::pathplanning::QToQSamplerPlanner>
        makeQToQSamplerPlanner(const SBLSetup& setup);

        /**
           @brief An SBL based point-to-point planner.

           @param setup [in] Setup for the planner.
        */
        static std::auto_ptr<rw::pathplanning::QToQPlanner>
        makeQToQPlanner(const SBLSetup& setup);

        /**
           @brief An SBL based point-to-tool-position planner.

           @param setup [in] Setup for the planner.
           @param ikSampler [in] Sampler of IK solutions for the target transform.
        */
        static std::auto_ptr<rw::pathplanning::QToTPlanner>
        makeQToTPlanner(
            const SBLSetup& setup,
            rw::pathplanning::QIKSamplerPtr ikSampler);

    private:
        SBLPlanner();
        SBLPlanner(const SBLPlanner&);
        SBLPlanner& operator=(const SBLPlanner&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
