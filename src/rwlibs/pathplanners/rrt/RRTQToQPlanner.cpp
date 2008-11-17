/*********************************************************************
 * RobWork Version 0.3
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
using namespace rw::common;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;

namespace
{
    typedef RRTNode<rw::math::Q> Node;
    typedef RRTTree<rw::math::Q> Tree;
    typedef rw::trajectory::QPath Path;

    const Q& getQ(Node* node) { return node->getValue(); }

    enum ExtendResult { Trapped, Reached, Advanced };

    class RRTStruct
    {
    public:
        RRTStruct(
            const PlannerConstraint& constraint,
            QSamplerPtr sampler,
            QMetricPtr metric,
            double extend)
            :
            constraint(constraint),
            sampler(sampler),
            metric(metric),
            extend(extend)
        {
            RW_ASSERT(sampler);
            RW_ASSERT(metric);
        }

        PlannerConstraint constraint;
        QSamplerPtr sampler;
        QMetricPtr metric;
        double extend;
    };

    bool inCollision(const RRTStruct& rrt, const Q& q)
    {
        return rrt.constraint.getQConstraint().inCollision(q);
    }

    // 'node' is known to be collision free, but 'b' is not.
    bool inCollision(
        const RRTStruct& rrt,
        Node* a,
        const Q& b)
    {
        return
            rrt.constraint.getQConstraint().inCollision(b) ||
            rrt.constraint.getQEdgeConstraint().inCollision(getQ(a), b);
    }

    // Brute-force nearest neighbor search.
    Node* nearestNeighbor(
        const RRTStruct& rrt,
        const Tree& tree,
        const Q& q)
    {
        const QMetric& metric = *rrt.metric;
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

    ExtendResult extend(
        const RRTStruct& rrt,
        Tree& tree,
        const Q& q,
        Node* qNearNode)
    {
        const Q& qNear = getQ(qNearNode);
        const Q delta = q - qNear;
        const double dist = rrt.metric->distance(delta);

        if (dist <= rrt.extend) {
            if (!inCollision(rrt, qNearNode, q)) {
                tree.add(q, qNearNode);
                return Reached;
            } else
                return Trapped;
        } else {
            const Q qNew = qNear + (rrt.extend / dist) * delta;
            if (!inCollision(rrt, qNearNode, qNew)) {
                tree.add(qNew, qNearNode);
                return Advanced;
            } else
                return Trapped;
        }
    }

    ExtendResult connect(
        const RRTStruct& rrt,
        Tree& tree,
        const Q& q)
    {
        Node* qNearNode = nearestNeighbor(rrt, tree, q);

        ExtendResult s = Advanced;
        while (s == Advanced) {
            s = extend(rrt, tree, q, qNearNode);
            if (s == Advanced) qNearNode = &tree.getLast();
        }
        return s;
    }

    ExtendResult growTree(
        const RRTStruct& rrt,
        Tree& tree,
        const Q& q)
    {
        Node* qNearNode = nearestNeighbor(rrt, tree, q);
        return extend(rrt, tree, q, qNearNode);
    }

    // Assuming that both trees have just been extended, retrieve the resulting
    // path.
    void getPath(const Tree& startTree, const Tree& goalTree, Path& result)
    {
        Path revPart;
        Tree::getRootPath(*startTree.getLast().getParent(), revPart);
        result.insert(result.end(), revPart.rbegin(), revPart.rend());
        Tree::getRootPath(goalTree.getLast(), result);
    }

    class RRTBasic : public QToQPlanner
    {
    public:
        RRTBasic(const RRTStruct& rrt) : _rrt(rrt) {}

    private:
        bool doQuery(
            const Q& start,
            const Q& goal,
            Path& result,
            const StopCriteria& stop)
        {
            if (inCollision(_rrt, start) || inCollision(_rrt, goal))
                return false;

            Tree startTree(start);
            Tree goalTree(goal);

            while (!stop.stop()) {
                const Q qAttr = _rrt.sampler->sample();
                if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

                // If both trees manage to connect, then return the resulting
                // path.
                if (growTree(_rrt, startTree, qAttr) == Reached &&
                    growTree(_rrt, goalTree, qAttr) == Reached)
                {
                    getPath(startTree, goalTree, result);
                    return true;
                }
            }

            return false;
        }

        RRTStruct _rrt;
    };

    class RRTConnect : public QToQPlanner
    {
    public:
        RRTConnect(
            const RRTStruct& rrt)
            :
            _rrt(rrt)
        {}

    private:
        bool doQuery(
            const Q& start,
            const Q& goal,
            Path& result,
            const StopCriteria& stop)
        {
            if (inCollision(_rrt, start) || inCollision(_rrt, goal))
                return false;

            Tree startTree(start);
            Tree goalTree(goal);
            Tree* treeA = &startTree;
            Tree* treeB = &goalTree;

            while (!stop.stop()) {

                const Q qAttr = _rrt.sampler->sample();
                if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

                if (growTree(_rrt, *treeA, qAttr) != Trapped &&
                    connect(_rrt, *treeB, getQ(&treeA->getLast())) == Reached)
                {
                    getPath(startTree, goalTree, result);
                    return true;
                }

                std::swap(treeA, treeB);
            }

            return false;
        }

        RRTStruct _rrt;
        bool _balanceTrees;
    };

    class RDTBalancedBidirectional : public QToQPlanner
    {
    public:
        RDTBalancedBidirectional(
            const RRTStruct& rrt,
            bool balanceTrees)
            :
            _rrt(rrt),
            _balanceTrees(balanceTrees)
        {}

    private:
        bool doQuery(
            const Q& start,
            const Q& goal,
            Path& result,
            const StopCriteria& stop)
        {
            if (inCollision(_rrt, start) || inCollision(_rrt, goal))
                return false;

            Tree startTree(start);
            Tree goalTree(goal);
            Tree* treeA = &startTree;
            Tree* treeB = &goalTree;

            while (!stop.stop()) {
                RW_ASSERT(!_balanceTrees || treeA->size() <= treeB->size());

                const Q qAttr = _rrt.sampler->sample();
                if (qAttr.empty()) RW_THROW("Sampler must always succeed.");

                if (connect(_rrt, *treeA, qAttr) != Trapped &&
                    connect(_rrt, *treeB, getQ(&treeA->getLast())) == Reached)
                {
                    getPath(startTree, goalTree, result);
                    return true;
                }

                if (!_balanceTrees || treeA->size() > treeB->size()) {
                    std::swap(treeA, treeB);
                }
            }

            return false;
        }

        RRTStruct _rrt;
        bool _balanceTrees;
    };
}

QToQPlannerPtr RRTQToQPlanner::makeBasic(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
{
    return ownedPtr(
        new RRTBasic(
            RRTStruct(constraint, sampler, metric, extend)));
}

QToQPlannerPtr RRTQToQPlanner::makeConnect(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
{
    return ownedPtr(
        new RRTConnect(
            RRTStruct(constraint, sampler, metric, extend)));
}

QToQPlannerPtr RRTQToQPlanner::makeBidirectional(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
{
    return ownedPtr(
        new RDTBalancedBidirectional(
            RRTStruct(constraint, sampler, metric, extend),
            true));
}

QToQPlannerPtr RRTQToQPlanner::makeBalancedBidirectional(
    const PlannerConstraint& constraint,
    QSamplerPtr sampler,
    QMetricPtr metric,
    double extend)
{
    return ownedPtr(
        new RDTBalancedBidirectional(
            RRTStruct(constraint, sampler, metric, extend),
            false));
}
