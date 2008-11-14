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

#include "Z3QToQPlanner.hpp"

#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;
using namespace rwlibs::pathplanners;

namespace
{
    typedef QPath Path;

    struct NodeValue
    {
        NodeValue(const Q& q, const Path& path)
            : q(q), path(path) {}

        Q q;
        Path path;
    };

    typedef RRTTree<NodeValue> Tree;
    typedef Tree::node_type Node;

    void getReverseRootPath(const Node& node, Path& result)
    {
        // We take some effort here to not include duplicate configurations.

        for (const Node* pos = &node; pos; pos = pos->getParent()) {
            const Q& q = pos->getValue().q;
            const Path& localPath = pos->getValue().path;

            result.push_back(q);

            typedef Path::const_reverse_iterator I;
            I begin = localPath.rbegin(); ++begin;
            I end = localPath.rend(); --end;

            result.insert(result.end(), begin, end);
        }
    }

    void getRootPath(const Node& node, Path& result)
    {
        Path path;
        getReverseRootPath(node, path);
        result.insert(result.end(), path.rbegin(), path.rend());
    }

    bool connectToTree(
        Node* from,
        const Tree& toTree,
        QToQPlanner& localPlanner,
        const StopCriteria& stop,
        Path& result)
    {
        BOOST_FOREACH(Node* to, toTree.getNodes()) {
            Path localPath;
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

            Path localPath;
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

    std::auto_ptr<Tree> makeTree(const Q& q)
    {
        Path path; path.push_back(q); path.push_back(q);
        return std::auto_ptr<Tree>(new Tree(NodeValue(q, path)));
    }
}

Z3QToQPlanner::Z3QToQPlanner(
    QSamplerPtr sampler,
    QToQPlannerPtr localPlanner,
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
    Path& result,
    const StopCriteria& stop)
{
    for (int repeat = 0;
         !stop.stop() && (_repeatCnt < 0 || repeat < _repeatCnt);
         repeat++)
    {
        std::auto_ptr<Tree> startTree = makeTree(start);
        std::auto_ptr<Tree> goalTree = makeTree(goal);

        std::vector<Q> startQs = sampleConfigurations(_nodeCnt, *_sampler);
        std::vector<Q> goalQs = sampleConfigurations(_nodeCnt, *_sampler);

        std::vector<Node*> startLayer(1, &startTree->getRoot());
        std::vector<Node*> goalLayer(1, &goalTree->getRoot());

        while (!startLayer.empty() || !goalLayer.empty()) {

            // Extend the start tree:
            std::vector<Node*> newStartLayer;
            BOOST_FOREACH(Node* from, startLayer) {
                Path path;
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
                Path path;
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
