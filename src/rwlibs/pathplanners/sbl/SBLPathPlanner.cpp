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

#include "SBLPathPlanner.hpp"
#include "SBLInternal.hpp"

#include <rw/pathplanning/QNormalizer.hpp>
#include <boost/foreach.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;

class SBLPathPlanner::Impl
{
public:
    Impl(
        QConstraintPtr constraint,
        const QNormalizer& normalizer)
        :
        normalizer(normalizer),
        normalizedConstraint(QConstraint::makeNormalized(constraint, normalizer))
    {}

    QNormalizer normalizer;
    Ptr<QConstraint> normalizedConstraint;
    SBLOptions options;
};

SBLPathPlanner::SBLPathPlanner(
    Ptr<QConstraint> constraint,
    const QNormalizer& normalizer)
    :
    _impl(new Impl(constraint, normalizer))
{}

SBLPathPlanner::~SBLPathPlanner() { delete _impl; }

bool SBLPathPlanner::doQuery(
    const Q& from,
    const Q& to,
    Path& result,
    const rw::pathplanning::StopCriteria& stop)
{
    const SBLInternal::Motion path =
        SBLInternal::findPath(
            _impl->normalizer.toNormalized(from),
            _impl->normalizer.toNormalized(to),
            *_impl->normalizedConstraint,
            _impl->options,
            stop);

    BOOST_FOREACH(const Q& q, path) {
        result.push_back(_impl->normalizer.fromNormalized(q));
    }
    return !path.empty();
}
