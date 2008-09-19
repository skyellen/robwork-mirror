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

#include "RRTQToQPlanner.hpp"

#include <climits>
#include <cfloat>
#include <algorithm>
#include <boost/foreach.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;

namespace
{
    typedef RRTNode<rw::math::Q> Node;
    typedef RRTTree<rw::math::Q> Tree;

    const Q& getQ(Node* node) { return node->getValue(); }

    // Brute-force nearest neighbor search.
    Node* nearestNeighbor(
        const QMetric& metric,
        const Tree& tree,
        const Q& q)
    {
        double minLength = DBL_MAX;
        Node* minNode = NULL;

        BOOST_FOREACH(Node* node, tree.getNodes()) {
            const double length = metric.distance(q, getQ(node));
            if (length < minLength) {
                minLength = length;
                minNode = node;
            }
        }

        RW_ASSERT(minNode);
        return minNode;
    }

    // 'node' is known to be collision free, but 'b' is not.
    bool inCollision(
        const PlannerConstraint& constraint,
        Node* a,
        const Q& b)
    {
        return
            constraint.getQConstraint().inCollision(b) ||
            constraint.getQEdgeConstraint().inCollision(getQ(a), b);
    }
}

RRTQToQPlanner::RRTQToQPlanner(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
    :
    _constraint(constraint),
    _sampler(sampler),
    _metric(metric),
    _extend(extend)
{}

RRTQToQPlanner::ExtendResult RRTQToQPlanner::extend(
    Tree& tree,
    const Q& q,
    Node* qNearNode)
{
    const Q& qNear = getQ(qNearNode);
    const Q delta = q - qNear;
    const double dist = _metric->distance(delta);

    if (dist <= _extend) {
        if (!inCollision(_constraint, qNearNode, q)) {
            tree.add(q, qNearNode);
            return Reached;
        } else
            return Trapped;
    } else {
        const Q qNew = qNear + (_extend / dist) * delta;
        if (!inCollision(_constraint, qNearNode, qNew)) {
            tree.add(qNew, qNearNode);
            return Advanced;
        } else
            return Trapped;
    }
}

RRTQToQPlanner::ExtendResult RRTQToQPlanner::connect(Tree& tree, const Q& q)
{
    Node* qNearNode = nearestNeighbor(*_metric, tree, q);

    ExtendResult s = Advanced;
    while (s == Advanced) {
        s = extend(tree, q, qNearNode);
        if (s == Advanced) qNearNode = &tree.getLast();
    }
    return s;
}

bool RRTQToQPlanner::doQuery(
    const Q& start,
    const Q& goal,
    Path& result,
    const StopCriteria& stop)
{
    if (_constraint.getQConstraint().inCollision(start) ||
        _constraint.getQConstraint().inCollision(goal))
        return false;

    Tree startTree(start);
    Tree goalTree(goal);
    Tree* treeA = &startTree;
    Tree* treeB = &goalTree;

    while (!stop.stop()) {
        const Q qAttr = _sampler->sample();
        if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

        Node* near = nearestNeighbor(*_metric, *treeA, qAttr);

        if (extend(*treeA, qAttr, near) != Trapped &&
            connect(*treeB, getQ(&treeA->getLast())) == Reached)
        {
            Path revPart;
            Tree::getRootPath(*startTree.getLast().getParent(), revPart);

            result.insert(result.end(), revPart.rbegin(), revPart.rend());
            Tree::getRootPath(goalTree.getLast(), result);
            return true;
        }

        std::swap(treeA, treeB);
    }

    return false;
}
