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

#include "SBLPlanner.hpp"
#include "SBLInternal.hpp"

#include <boost/foreach.hpp>

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

QToQSamplerPlannerPtr
SBLPlanner::makeQToQSamplerPlanner(const SBLSetup& setup)
{
    return ownedPtr(new SBLQToQSamplerPlanner(setup));
}

QToQPlannerPtr
SBLPlanner::makeQToQPlanner(const SBLSetup& setup)
{
    return QToQPlanner::make(
        makeQToQSamplerPlanner(setup));
}

rw::pathplanning::QToTPlannerPtr
SBLPlanner::makeQToTPlanner(
    const SBLSetup& setup,
    QIKSamplerPtr ikSampler)
{
    return QToTPlanner::make(makeQToQSamplerPlanner(setup), ikSampler);
}
