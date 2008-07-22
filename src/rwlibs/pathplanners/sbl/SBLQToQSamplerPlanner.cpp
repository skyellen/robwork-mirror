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

#include "SBLQToQSamplerPlanner.hpp"
#include "SBLInternal.hpp"

#include <rw/pathplanning/QNormalizer.hpp>
#include <boost/foreach.hpp>

using namespace rwlibs::pathplanners;
using namespace rw::pathplanning;
using namespace rw::math;
using namespace rw::common;
using namespace rw::models;

class SBLQToQSamplerPlanner::Impl
{
public:
    Impl(
        QConstraintPtr constraint,
        QEdgeConstraintPtr edge,
        QExpandPtr expansion)
        :
        constraint(constraint),
        edge(edge),
        expansion(expansion)
    {}

    QConstraintPtr constraint;
    QEdgeConstraintPtr edge;
    QExpandPtr expansion;

    SBLOptions options;
};

SBLQToQSamplerPlanner::SBLQToQSamplerPlanner(
    QConstraintPtr constraint,
    QEdgeConstraintPtr edge,
    QExpandPtr expansion)
    :
    _impl(new Impl(constraint, edge, expansion))
{}

SBLQToQSamplerPlanner::~SBLQToQSamplerPlanner() { delete _impl; }

bool SBLQToQSamplerPlanner::doQuery(
    const Q& from,
    QSampler& to,
    Path& result,
    const StopCriteria& stop)
{
    const SBLInternal::Motion path =
        SBLInternal::findApproach(
            from,
            Q(),
            SBLInternal::Motion(),
            to,
            *_impl->constraint,
            *_impl->edge,
            *_impl->expansion,
            _impl->options,
            stop);

    BOOST_FOREACH(const Q& q, path) {
        result.push_back(q);
    }

    return !path.empty();
}

std::auto_ptr<QToQSamplerPlanner> SBLQToQSamplerPlanner::make(
    QConstraintPtr constraint,
    QEdgeConstraintPtr edge,
    QExpandPtr expansion)
{
    typedef std::auto_ptr<QToQSamplerPlanner> T;
    return T(new SBLQToQSamplerPlanner(constraint, edge, expansion));
}

std::auto_ptr<QToQSamplerPlanner> SBLQToQSamplerPlanner::make(
    QConstraintPtr constraint,
    QEdgeConstraintPtr edge,
    DevicePtr device)
{
    const double expandRatio = 0.2;
    return make(
        constraint,
        edge,
        QExpand::makeUniformBox(
            device->getBounds(),
            expandRatio));
}

std::auto_ptr<QToQSamplerPlanner> SBLQToQSamplerPlanner::make(
    QConstraintPtr constraint,
    DevicePtr device)
{
    return make(
        constraint,
        QEdgeConstraint::makeDefault(constraint, device),
        device);
}

// #include <rw/pathplanning/QToTPlanner.hpp>
// #include <rw/loaders/WorkCellLoader.hpp>
// #include <rw/models/WorkCell.hpp>
// #include <rw/kinematics/State.hpp>
// #include <rw/invkin/ResolvedRateSolver.hpp>
// using namespace rw::loaders;
// using namespace rw::invkin;
// using namespace rw::kinematics;
// namespace
// {
//     void f()
//     {
//         QConstraintPtr constraint = QConstraint::makeFixed(false);

//         QToQSamplerPlannerPtr planner =
//             SBLQToQSamplerPlanner::make(
//                 constraint,
//                 QEdgeConstraint::makeDefault(
//                     constraint,
//                     WorkCellLoader::load("bar")->findDevice("foo")),
//                 QExpand::makeUniformBox(
//                     QExpand::QBox(),
//                     QExpand::QBox()));

//         QToTPlannerPtr ptr = QToTPlanner::make(
//             planner,
//             new ResolvedRateSolver(0, State()),
//             0,
//             State(),
//             12);
//     }
// }
