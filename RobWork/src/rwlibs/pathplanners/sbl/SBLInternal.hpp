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


#ifndef RWLIBS_PATHPLANNERS_SBL_SBLINTERNAL_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLINTERNAL_HPP

/**
   @file SBLInternal.hpp

   @brief SBL path planner.
*/

#include "SBLSetup.hpp"

namespace rw { namespace math { class Q; } }
namespace rw { namespace pathplanning { class StopCriteria; } }
namespace rw { namespace pathplanning { class QSampler; } }

namespace rwlibs { namespace pathplanners {

    class SBLInternal
    {
    public:
        typedef std::vector<rw::math::Q> Motion;

        /**
           @brief The options stored within the setup.
        */
        static SBLOptions getOptions(const SBLSetup& setup) { return setup.options; }

        /**
           A general path planner call that (depending on the arguments) can find
           standard paths, approach paths, retract paths, and connection paths.

           The planner assumes that all of the involved configurations are
           normalized.

           The planner runs until a path is found or an external thread assigns \b
           stop a value of true.

           @param from [in] Collision free start configuration (or the empty path is
           returned) or NULL (fromSampler is used to compute a configuration).

           @param to [in] Collision free goal configuration (or the empty path is
           returned) or NULL (toSampler is used to compute a configuration).

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
            const SBLOptions& setup,
            const rw::pathplanning::StopCriteria& stop);

        /**
           Standard path planning.
        */
        static Motion findPath(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const SBLOptions& setup,
            const rw::pathplanning::StopCriteria& stop);

        /**
           Approach planning.
        */
        static Motion findApproach(
            const rw::math::Q& from,
            const rw::math::Q& to,
            const Motion& toSamples,
            rw::pathplanning::QSampler& toSampler,
            const SBLOptions& setup,
            const rw::pathplanning::StopCriteria& stop);
    };

}} // end namespaces

#endif // end include guard
