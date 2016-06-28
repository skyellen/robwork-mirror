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
        RRTStruct(const PlannerConstraint& constraint,
			QSampler::Ptr sampler,
			QMetric::Ptr metric,
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
		QSampler::Ptr sampler;
		QMetric::Ptr metric;
        double extend;
    };

    bool inCollision(const RRTStruct& rrt, const Q& q)
    {
        return rrt.constraint.getQConstraint().inCollision(q);
    }

    // 'node' is known to be collision free, but 'b' is not.
    bool inCollision(const RRTStruct& rrt,
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

    ExtendResult extend(const RRTStruct& rrt,
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
            } else {
                return Trapped;
            }
        } else {
            const Q qNew = qNear + (rrt.extend / dist) * delta;
            if (!inCollision(rrt, qNearNode, qNew)) {
                tree.add(qNew, qNearNode);
                return Advanced;
            } else {
                return Trapped;
            }
        }
    }

    ExtendResult connect(
        const RRTStruct& rrt,
        Tree& tree,
        const Q& q)
    {
        Node* qNearNode = nearestNeighbor(rrt, tree, q);

        ExtendResult s = Advanced;
		bool hasAdvanced = false;
        while (s == Advanced) {
            s = extend(rrt, tree, q, qNearNode);
			if (s == Advanced) {
				qNearNode = &tree.getLast();
				hasAdvanced = true;
			}
        } 

		if (s == Trapped && hasAdvanced)
			return Advanced;
		else
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
			if (inCollision(_rrt, start)) {
				std::cout<<"Start is in collision"<<std::endl;
                return false;
			}

			if (inCollision(_rrt, goal)) {
				std::cout<<"Goal is in collision"<<std::endl;
                return false;
			}

			if (!_rrt.constraint.getQEdgeConstraint().inCollision(start, goal)) {
				result.push_back(start);
				result.push_back(goal);
				return true;
			}

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
			if (inCollision(_rrt, start)) {
				std::cout<<"Start is in collision"<<std::endl;
                return false;
			}

			if (inCollision(_rrt, goal)) {
				std::cout<<"Goal is in collision"<<std::endl;
                return false;
			}

			if (!_rrt.constraint.getQEdgeConstraint().inCollision(start, goal)) {
				result.push_back(start);
				result.push_back(goal);
				return true;
			}



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
			if (inCollision(_rrt, start)) {
				std::cout<<"Start is in collision"<<std::endl;
                return false;
			}

			if (inCollision(_rrt, goal)) {
				std::cout<<"Goal is in collision"<<std::endl;
                return false;
			}

			if (!_rrt.constraint.getQEdgeConstraint().inCollision(start, goal)) {
				result.push_back(start);
				result.push_back(goal);
				return true;
			}


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

QToQPlanner::Ptr RRTQToQPlanner::makeBasic(const PlannerConstraint& constraint,
										 QSampler::Ptr sampler,
										 QMetric::Ptr metric,
                                         double extend)
{
    return ownedPtr(
        new RRTBasic(
            RRTStruct(constraint, sampler, metric, extend)));
}

QToQPlanner::Ptr RRTQToQPlanner::makeConnect(
    const PlannerConstraint& constraint,
	QSampler::Ptr sampler,
	QMetric::Ptr metric,
    double extend)
{
    return ownedPtr(
        new RRTConnect(
            RRTStruct(constraint, sampler, metric, extend)));
}

QToQPlanner::Ptr RRTQToQPlanner::makeBidirectional(
    const PlannerConstraint& constraint,
	QSampler::Ptr sampler,
	QMetric::Ptr metric,
    double extend)
{
    return ownedPtr(
        new RDTBalancedBidirectional(
            RRTStruct(constraint, sampler, metric, extend),
            true));
}

QToQPlanner::Ptr RRTQToQPlanner::makeBalancedBidirectional(
    const PlannerConstraint& constraint,
	QSampler::Ptr sampler,
	QMetric::Ptr metric,
    double extend)
{
    return ownedPtr(
        new RDTBalancedBidirectional(
            RRTStruct(constraint, sampler, metric, extend),
            false));
}
