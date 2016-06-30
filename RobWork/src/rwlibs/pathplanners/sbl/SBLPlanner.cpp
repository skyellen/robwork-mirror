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


#include "SBLPlanner.hpp"
#include "SBLInternal.hpp"
#include "SBLSetup.hpp"

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;

namespace
{
    class SBLQToQSamplerPlanner : public rw::pathplanning::QToQSamplerPlanner
    {
    public:
        SBLQToQSamplerPlanner(const SBLSetup& setup)
            : _options(SBLInternal::getOptions(setup))
        {}

    private:
        bool doQuery(
            const Q& from,
            QSampler& to,
            QPath& result,
            const StopCriteria& stop)
        {
            const SBLInternal::Motion path =
                SBLInternal::findApproach(
                    from,
                    Q(),
                    SBLInternal::Motion(),
                    to,
                    _options,
                    stop);

            result.insert(result.end(), path.begin(), path.end());

            return !path.empty();
        }

    private:
        SBLOptions _options;
    };
}

QToQSamplerPlanner::Ptr SBLPlanner::makeQToQSamplerPlanner(const SBLSetup& setup)
{
    return ownedPtr(new SBLQToQSamplerPlanner(setup));
}

QToQPlanner::Ptr SBLPlanner::makeQToQPlanner(const SBLSetup& setup)
{
    return QToQPlanner::make(
        makeQToQSamplerPlanner(setup));
}

rw::pathplanning::QToTPlanner::Ptr SBLPlanner::makeQToTPlanner(const SBLSetup& setup,
															   rw::common::Ptr<QIKSampler> ikSampler)
{
    return QToTPlanner::make(makeQToQSamplerPlanner(setup), ikSampler);
}
