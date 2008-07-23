/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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
#include <rw/math/MetricUtil.hpp>

#include <limits.h>
#include <float.h>

using namespace rw;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;

namespace
{
    //! The stepsize to use
    const double EPSILON = 0.5;
}

RRTQToQPlanner::RRTQToQPlanner(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler)
    :
    _constraint(constraint),
    _sampler(sampler)
{}

double RRTQToQPlanner::d(const Q& a, const Q& b) const
{
    // Calculates the distance as the euclidian distance in joint space
    return MetricUtil::dist2(a, b);
}

const RRTQToQPlanner::Node* RRTQToQPlanner::nearestNeighbor(
    const Tree& tree, const Q& q) const
{
    // Precondition: Tree is not empty and q is a valid joint configuration
    RW_ASSERT(tree.size() != 0);

    double minLength = DBL_MAX;
    const Node* min = NULL;

    for (unsigned int i = 0; i < tree.size(); i++) {
        double length = d(q, tree[i]->getQ());

        if (length < minLength) {
            minLength = length;
            min = tree.at(i);
        }
    }

    // Postcondition: a node was found and distance < DBL_MAX
    RW_ASSERT(min != NULL);
    RW_ASSERT(minLength < DBL_MAX);

    return min;
}

RRTQToQPlanner::ExtendResult RRTQToQPlanner::extend(
    Tree& tree,
    const Q& q,
    const Node* qNearNode)
{
    // Precondition: tree is not empty and q is a valid configuration
    RW_ASSERT(tree.size() != 0);

    const Q& qNear = qNearNode->getQ();

    const Q delta = q - qNear;
    const double dist = delta.norm2();

    if (dist <= EPSILON) {
        if (!inCollision(qNear, q)) {
            tree.push_back(new Node(q, qNearNode));
            return REACHED;
        } else
            return TRAPPED;
    } else {
        // Take a step toward q.
        const Q qNew = qNear + (EPSILON / dist) * delta;
        if (!inCollision(qNear, qNew)) {
            tree.push_back(new Node(qNew, qNearNode));
            return ADVANCED;
        } else
            return TRAPPED;
    }
}

bool RRTQToQPlanner::inCollision(const Q& a, const Q& b)
{
    return
        _constraint.getQConstraint().inCollision(b) ||
        _constraint.getQEdgeConstraint().inCollision(a, b);
}

RRTQToQPlanner::ExtendResult RRTQToQPlanner::connect(Tree& tree, const Q& q)
{
    // Precondition: tree is not empty and q is a valid configuration
    RW_ASSERT(tree.size() != 0);

    ExtendResult s;

    const Node* qNearNode = nearestNeighbor(tree, q);

    do {
        s = extend(tree, q, qNearNode);
        if (s == ADVANCED)
            qNearNode = tree.back();
    } while (s == ADVANCED);

    return s;
}

bool RRTQToQPlanner::doQuery(
    const Q& qInit,
    const Q& qGoal,
    Path& path,
    const StopCriteria& stop)
{
    if (_constraint.getQConstraint().inCollision(qInit) ||
        _constraint.getQConstraint().inCollision(qGoal))
        return false;

    Tree tree1;
    Tree tree2;
    Tree* Ta = &tree1;
    Tree* Tb = &tree2;

    Ta->push_back(new Node(qInit, NULL));
    Tb->push_back(new Node(qGoal, NULL));

    const unsigned int K = 10000;
    for (unsigned k = 1; k < K && !stop.stop(); k++) {
        const Q qRand = _sampler->sample();
        if (qRand.empty()) RW_THROW("No sample found.");

        const Node* qNearNode = nearestNeighbor(*Ta, qRand);

        if (extend(*Ta, qRand, qNearNode) != TRAPPED) {
            if (connect(*Tb, Ta->back()->getQ()) == REACHED) {
                const Node* nodeIterator;

                nodeIterator = tree1.back();
                while (nodeIterator != NULL){
                    path.push_front(nodeIterator->getQ());
                    nodeIterator = nodeIterator->getParent();
                }

                nodeIterator = tree2.back()->getParent();

                while(nodeIterator != NULL){
                    path.push_back(nodeIterator->getQ());
                    nodeIterator = nodeIterator->getParent();
                }

                for (unsigned i1 = 0; i1 < tree1.size(); i1++) {
                    delete tree1[i1];
                }

                for (unsigned i2 = 0; i2 < tree2.size(); i2++) {
                    delete tree2[i2];
                }

                return true;
            }
        }

        // Swap pointers
        Tree* temp = Ta;
        Ta = Tb;
        Tb = temp;
    }

    return false;
}
