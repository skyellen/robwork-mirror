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

#ifndef RWLIBS_PATHPLANNERS_SBL_SBLINTERNAL_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLINTERNAL_HPP

/**
   @file SBLInternal.hpp

   @brief SBL path planner.
*/

#include "SBLOptions.hpp"

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/StopCriteria.hpp>
#include <rw/math/Q.hpp>

namespace rwlibs { namespace pathplanners {

    class SBLInternal
    {
    public:
        typedef std::vector<rw::math::Q> Motion;

        /**
           A general path planner call that (depending on the arguments) can find
           standard paths, approach paths, retract paths, and connection paths.

           The planner assumes that all of the involved configurations are
           normalized.

           The planner runs until a path is found or an external thread assigns \b
           stop a value of true.

           @param from [in] Collision free start configuration (or the empty path is
           returned).

           @param to [in] Collision free goal configuration (or the empty path is
           returned).

           @param fromSamples [in] Other samples to insert for the start region.

           @param toSamples [in] Other samples to insert for the goal region.

           @param fromSampler [in] Sampler (possibly empty) for start region.

           @param toSampler [in] Sampler (possibly empty) for goal region.

           @param constraint [in] The collision constraint for which planning is done.

           @param options [in] Options for the SBL planner.

           @param stop [in] Stop the planner when this function object returns true.
        */
        static Motion findConnection(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const Motion& fromSamples,
            const Motion& toSamples,
            rw::pathplanning::QSampler& fromSampler,
            rw::pathplanning::QSampler& toSampler,
            const rw::pathplanning::QConstraint& constraint,
            const SBLOptions& options,
            rw::pathplanning::StopCriteriaPtr stop);

        /**
           Standard path planning.
        */
        static Motion findPath(
            const rw::math::Q& from,
            const rw::math::Q& to,
            rw::pathplanning::QConstraint& constraint,
            const SBLOptions& options,
            rw::pathplanning::StopCriteriaPtr stop);

        /**
           Approach planning.
        */
        static Motion findApproach(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const Motion& toSamples,
            rw::pathplanning::QSampler& toSampler,
            rw::pathplanning::QConstraint& constraint,
            const SBLOptions& options,
            rw::pathplanning::StopCriteriaPtr stop);
    };

}} // end namespaces

#endif // end include guard
