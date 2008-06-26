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

#include "RRTPathPlanner.hpp"
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

RRTPathPlanner::RRTPathPlanner(
    rw::pathplanning::QConstraintPtr constraint,
    rw::pathplanning::QSamplerPtr sampler,
    double resolution)
    :
    _constraint(constraint),
    _sampler(sampler),
    _resolution(resolution),
    _lineplanner(_constraint, resolution)
{}

RRTPathPlanner::RRTPathPlanner(
    WorkCell* workcell,
    Device* device,
    CollisionDetector* detector,
    const rw::kinematics::State& state,
    double resolution)
    :
    _constraint(QConstraint::make(detector, device, state)),
    _sampler(QSampler::makeUniform(*device)),
    _resolution(resolution),
    _lineplanner(_constraint, resolution)
{}

double RRTPathPlanner::d(const Q& a, const Q& b) const
{
    // Calculates the distance as the euclidian distance in joint space
    return MetricUtil::dist2(a, b);
}

const RRTPathPlanner::Node* RRTPathPlanner::nearestNeighbor(
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

RRTPathPlanner::ExtendResult RRTPathPlanner::extend(
    Tree& tree,
    const Q& q,
    const Node* qNearNode)
{
    // Precondition: tree is not empty and q is a valid configuration
    RW_ASSERT(tree.size() != 0);

    const Q& qNear = qNearNode->getQ();

    if ((q - qNear).norm2() <= EPSILON) {
        if (!collides(qNear, q)) {
            tree.push_back(new Node(q, qNearNode));
            return REACHED;
        } else
            return TRAPPED;
    } else {
        // Find the distance vector
        Q delta = q - qNear;

        // Find the direction of the distance vector
        delta /= (delta).norm2();

        // Take a step toward q
        delta *= EPSILON;

        Q qNew = qNear + delta;
        if (!collides(qNear, qNew)) {
            tree.push_back(new Node(qNew, qNearNode));
            return ADVANCED;
        } else
            return TRAPPED;
    }
}

bool RRTPathPlanner::collides(const Q& qNear, const Q& qNew)
{
    std::list<Q> dummy;
    return !_lineplanner.query(qNear, qNew, dummy);
}

RRTPathPlanner::ExtendResult RRTPathPlanner::connect(Tree& tree, const Q& q)
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

bool RRTPathPlanner::solve(
    const Q& qInit,
    const Q& qGoal,
    Path& path,
    StopCriteriaPtr stop)
{
    if (_constraint->inCollision(qInit)) return false;
    if (_constraint->inCollision(qGoal)) return false;

    Tree tree1;
    Tree tree2;
    Tree* Ta = &tree1;
    Tree* Tb = &tree2;

    Ta->push_back(new Node(qInit, NULL));
    Tb->push_back(new Node(qGoal, NULL));

    const unsigned int K = 10000;
    for (unsigned k = 1; k < K && !stop->stop(); k++) {
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
