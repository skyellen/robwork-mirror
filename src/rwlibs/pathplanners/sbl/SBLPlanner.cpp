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
using namespace rw::trajectory;

namespace
{
    class SBLQToQSamplerPlanner : public rw::pathplanning::QToQSamplerPlanner
    {
    public:
        SBLQToQSamplerPlanner(const SBLSetup& setup)
            : _setup(setup)
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
                    _setup,
                    stop);

            result.insert(result.end(), path.begin(), path.end());

            return !path.empty();
        }

    private:
        SBLSetup _setup;
    };
}

std::auto_ptr<QToQSamplerPlanner>
SBLPlanner::makeQToQSamplerPlanner(const SBLSetup& setup)
{
    typedef std::auto_ptr<QToQSamplerPlanner> T;
    return T(new SBLQToQSamplerPlanner(setup));
}

std::auto_ptr<QToQPlanner>
SBLPlanner::makeQToQPlanner(const SBLSetup& setup)
{
    return QToQPlanner::make(
        makeQToQSamplerPlanner(setup));
}

std::auto_ptr<rw::pathplanning::QToTPlanner>
SBLPlanner::makeQToTPlanner(
    const SBLSetup& setup,
    QIKSamplerPtr ikSampler)
{
    return QToTPlanner::make(makeQToQSamplerPlanner(setup), ikSampler);
}
