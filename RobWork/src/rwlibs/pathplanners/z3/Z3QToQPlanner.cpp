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


#include "Z3QToQPlanner.hpp"

#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>

using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::pathplanners;

namespace
{
     struct NodeValue
    {
        NodeValue(const Q& q, const QPath& path)
            : q(q), path(path) {}

        Q q;
        QPath path;
    };

    typedef RRTTree<NodeValue> Tree;
    typedef Tree::node_type Node;

    void getReverseRootPath(const Node& node, QPath& result)
    {
        // We take some effort here to not include duplicate configurations.

        for (const Node* pos = &node; pos; pos = pos->getParent()) {
            const Q& q = pos->getValue().q;
            const QPath& localPath = pos->getValue().path;

            result.push_back(q);

            typedef QPath::const_reverse_iterator I;
            I begin = localPath.rbegin(); ++begin;
            I end = localPath.rend(); --end;

            result.insert(result.end(), begin, end);
        }
    }

    void getRootPath(const Node& node, QPath& result)
    {
        QPath path;
        getReverseRootPath(node, path);
        result.insert(result.end(), path.rbegin(), path.rend());
    }

    bool connectToTree(
        Node* from,
        const Tree& toTree,
        QToQPlanner& localPlanner,
        const StopCriteria& stop,
        QPath& result)
    {
        BOOST_FOREACH(Node* to, toTree.getNodes()) {
            QPath localPath;
            const bool ok = localPlanner.query(
                from->getValue().q,
                to->getValue().q,
                localPath,
                stop);

            // If we found a connection:
            if (ok) {
                getRootPath(*from, result);
                result.insert(
                    result.end(), localPath.begin(), localPath.end());
                getReverseRootPath(*to, result);
                return true;
            }
        }
        return false;
    }

    void connectToQs(
        Node* from,
        Tree& tree,
        std::vector<Q>& qs,
        QToQPlanner& localPlanner,
        const StopCriteria& stop,
        std::vector<Node*>& layer)
    {
        std::vector<Node*> newStartLayer;

        std::vector<size_t> eraseIndices;
        for (size_t i = 0; i < qs.size(); i++) {
            const Q& q = qs[i];

            QPath localPath;
            const bool ok = localPlanner.query(
                from->getValue().q, q, localPath, stop);

            if (ok) {
                // Insert a new node in the tree.
                tree.add(NodeValue(q, localPath), from);

                // Add the new node to new layer.
                layer.push_back(&tree.getLast());

                // Mark the configuration to be erased.
                eraseIndices.push_back(i);
            }
        }

        // Erase the configurations that we used.
        BOOST_FOREACH(
            size_t index,
            std::make_pair(eraseIndices.rbegin(), eraseIndices.rend()))
        {
            qs.erase(qs.begin() + index);
        }
    }

    std::vector<Q> sampleConfigurations(int cnt, QSampler& sampler)
    {
        std::vector<Q> result;
        for (int i = 0; i < cnt; i++)
            result.push_back(sampler.sample());
        return result;
    }

    rw::common::Ptr<Tree> makeTree(const Q& q)
    {
        QPath path;
        path.push_back(q); 
        path.push_back(q);
        return rw::common::ownedPtr<Tree>(new Tree(NodeValue(q, path)));
    }
}

Z3QToQPlanner::Z3QToQPlanner(QSampler::Ptr sampler,
	QToQPlanner::Ptr localPlanner,
    int nodeCnt,
    int repeatCnt)
    :
    _sampler(sampler),
    _localPlanner(localPlanner),
    _nodeCnt(nodeCnt),
    _repeatCnt(repeatCnt)
{
    if (_nodeCnt < 0) _nodeCnt = 20;

    RW_ASSERT(_sampler);
    RW_ASSERT(_localPlanner);
}

bool Z3QToQPlanner::doQuery(
    const Q& start,
    const Q& goal,
    QPath& result,
    const StopCriteria& stop)
{
    for (int repeat = 0;
         !stop.stop() && (_repeatCnt < 0 || repeat < _repeatCnt);
         repeat++)
    {
        rw::common::Ptr<Tree> startTree = makeTree(start);
        rw::common::Ptr<Tree> goalTree = makeTree(goal);

        std::vector<Q> startQs = sampleConfigurations(_nodeCnt, *_sampler);
        std::vector<Q> goalQs = sampleConfigurations(_nodeCnt, *_sampler);

        std::vector<Node*> startLayer(1, &startTree->getRoot());
        std::vector<Node*> goalLayer(1, &goalTree->getRoot());

        while (!startLayer.empty() || !goalLayer.empty()) {

            // Extend the start tree:
            std::vector<Node*> newStartLayer;
            BOOST_FOREACH(Node* from, startLayer) {
                QPath path;
                const bool ok = connectToTree(
                    from, *goalTree, *_localPlanner, stop, path);
                if (ok) {
                    result.insert(result.end(), path.begin(), path.end());
                    return true;
                }
                connectToQs(
                    from, *startTree, startQs, *_localPlanner, stop, newStartLayer);
            }
            startLayer = newStartLayer;

            // Extend the goal tree:
            std::vector<Node*> newGoalLayer;
            BOOST_FOREACH(Node* from, goalLayer) {
                QPath path;
                const bool ok = connectToTree(
                    from, *startTree, *_localPlanner, stop, path);
                if (ok) {
                    result.insert(result.end(), path.rbegin(), path.rend());
                    return true;
                }
                connectToQs(
                    from, *goalTree, goalQs, *_localPlanner, stop, newGoalLayer);
            }
            goalLayer = newGoalLayer;
        }
    }
    return false;
}
