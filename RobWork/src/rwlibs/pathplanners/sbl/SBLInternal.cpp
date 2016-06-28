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


#include "SBLInternal.hpp"

#include <rw/pathplanning/QEdgeConstraintIncremental.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Random.hpp>
#include <rw/math/MetricUtil.hpp>

#include <cmath>
#include <cassert>
#include <queue>
#include <algorithm>
#include <utility>
#include <iterator>
#include <map>
#include <boost/foreach.hpp>

#define NS rwlibs::pathplanners::SBLInternal
typedef NS::Motion Motion;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;
using namespace rw::math;

//----------------------------------------------------------------------
// General utilities

namespace
{
    template <typename T>
    void reverseOnto(const std::vector<T>& vals, std::vector<T>& result)
    {
        result.insert(result.end(), vals.rbegin(), vals.rend());
    }
}

//----------------------------------------------------------------------
// Nodes

namespace
{
    enum NodeType { StartTree, GoalTree, UnknownTree };

    class Node
    {
        // A node stores a parent pointer and information on the collision
        // detection done for that edge. To be able to build our spatial indexes
        // add a helper field to mark whether the node belongs to the start
        // or the goal tree. This helper field is modified by external routines
        // and is _irrelevant_ to the invariant of the Node class. (The
        // external routines are entirely responsible for assigning values to
        // this field.)

    public:
        const Q q;
        Node* parent;
		QEdgeConstraintIncremental::Ptr edge;

    public:
        NodeType type;

    public:
        Node(const Q& q, Node* parent, const QEdgeConstraintIncremental& edge)
            :
            q(q),
            parent(parent),
            edge(edge.instance(q, parent ? parent->q : q)),
            type(UnknownTree)
        {}

    private:
        Node(const Node& other);
        Node& operator=(const Node& other);
    };

    double nodeDistance(
        const QMetric& metric, const Node& na, const Node& nb)
    {
        return metric.distance(na.q, nb.q);
    }
}

//----------------------------------------------------------------------
// Priority queues of nodes

namespace
{
    class CompareNode
    {
    public:
        bool operator()(const Node* a, const Node* b)
        {
            // Return 'true' if 'a' should come later than 'b'.
            return a->edge->inCollisionCost() < b->edge->inCollisionCost();
        }
    };

    typedef std::priority_queue<
        Node*,
        std::vector<Node*>,
        CompareNode> PriorityQueue;
}

//----------------------------------------------------------------------
// Spatial indexes

namespace
{
    typedef std::pair<int, int> IndexPair;

    IndexPair randomIndexPair(int dim)
    {
        // Here one might want to force the values of the pair to be
        // non-equal (if possible).
        return std::make_pair(
            Random::ranI(0, dim),
			Random::ranI(0, dim));
    }

    class SpatialIndex
    {
        int mappingSize;
        int nodeCount;
        SBLOptions options;

        typedef std::vector<Node*> Cell;

        typedef std::map<IndexPair, Cell> TreeMapping;
        TreeMapping treeMap;

        std::pair<double, double> xrange;
        std::pair<double, double> yrange;

        int xindex;
        int yindex;

        std::vector<Cell*> cellsInUse;

    public:
        SpatialIndex(const SBLOptions& options)
            :
            mappingSize(1),
            nodeCount(0),
            options(options)
        {
            // No good default values exist here.
            xindex = -1;
            yindex = -1;
            xrange = std::make_pair(0, 1);
            yrange = std::make_pair(0, 1);
        }

        int size() const { return nodeCount; }

    private:
        int arraySize(int tree_size) const
        {
            return (int)floor(sqrt((double)tree_size / options.nodesPerCell));
        }

    public:
        typedef std::vector<Node*> NodeVector;

    private:
        typedef NodeVector::const_iterator NI;

    public:
        void reset(
            const NodeVector& nodes,
            const IndexPair& pair)
        {
            cellsInUse.clear();

            mappingSize = arraySize((int)nodes.size());

            clearMap();

            xindex = pair.first;
            yindex = pair.second;

            setXYRanges(nodes);

            nodeCount = 0;
            for (NI p = nodes.begin(); p != nodes.end(); ++p)
                addNode(*p);
        }

    private:
        void setXYRanges(const NodeVector& nodes)
        {
            double xlow = 1;
            double xhigh = 0;
            double ylow = 1;
            double yhigh = 0;

            for (NI p = nodes.begin(); p != nodes.end(); ++p) {
                const Q& q = (*p)->q;

                double vx = q[xindex];
                double vy = q[yindex];

                if (vx < xlow) xlow = vx;
                if (vx > xhigh) xhigh = vx;
                if (vy < ylow) ylow = vy;
                if (vy > yhigh) yhigh = vy;
            }
            xrange.first = xlow;
            xrange.second = xhigh;
            yrange.first = ylow;
            yrange.second = yhigh;
        }

    private:
        void clearMap()
        {
            treeMap.clear();
        }

    public:
        void addNode(Node* node)
        {
            Cell& cell = cellOfNode(node);
            if (cell.empty()) cellsInUse.push_back(&cell);
            cell.push_back(node);
            nodeCount++;
        }

    private:
        IndexPair indexPairOf(Node* node) const
        {
            RW_ASSERT(node);

            double vx = node->q[xindex];
            double vy = node->q[yindex];

            double xlow = xrange.first;
            double xhigh = xrange.second;
            double ylow = yrange.first;
            double yhigh = yrange.second;

            int x = (int)floor(mappingSize * (vx - xlow) / (xhigh - xlow));
            int y = (int)floor(mappingSize * (vy - ylow) / (yhigh - ylow));

            return std::make_pair(x, y);
        }

    private:
        Cell& cellOfNode(Node* node)
        {
            return getCell(indexPairOf(node));
        }

    private:
        Cell& getCell(const IndexPair& pair)
        {
            return treeMap[pair];
        }

    private:
        static Node* randomNodeFromCell(const Cell& cell)
        {
            RW_ASSERT(!cell.empty());
            return
                cell.at(
                    Random::ranI(0, (int)cell.size()));
        }

    private:
        Node* randomNodeStandard() const
        {
            return
                randomNodeFromCell(
                    *cellsInUse.at(
                    	Random::ranI(0, (int)cellsInUse.size())));
        }

    private:
        Node* randomNodeSkewed() const
        {
            // With more elaborate data structures this could be done in
            // logarithmic time. In practice however the number of cells is
            // typically small.

            const double pos = Random::ran(0, 1);
            const int cellCount = (int)cellsInUse.size();

            if (cellCount == 1)
                return randomNodeFromCell(*cellsInUse.at(0));

            double sum = 0;
            for (int i = 0; i < cellCount; i++) {
                const Cell& cell = *cellsInUse[i];
                sum +=
                    (double)(nodeCount - cell.size()) /
                    (double)(nodeCount * (cellCount - 1));
                if (sum > pos)
                    return randomNodeFromCell(cell);
            }

            RW_ASSERT(0);
            return 0;
        }

    private:
        Node* randomNodeUniform() const
        {
            int choice = Random::ranI(0, nodeCount);

            int acc = 0;
            for (int i = 0; i < (int)cellsInUse.size(); i++) {
                acc += (int)cellsInUse[i]->size();
                if (acc > choice) {
                    return cellsInUse[i]->at(acc - choice - 1);
                }
            }
            RW_ASSERT(0);
            return 0;
        }

    public:
        Node* randomNode() const
        {
            RW_ASSERT(!cellsInUse.empty());
            if (cellsInUse.size() < 50) {
                return randomNodeSkewed();
            } else {
                return randomNodeStandard();
            }
        }

    private:
        Node* nearestNodeFromCell(const Cell& cell, Node* node) const
        {
            RW_ASSERT(!cell.empty());

            NI p = cell.begin();
            double min = nodeDistance(*options.metric, *node, **p);
            NI minNode = p;

            for (++p; p != cell.end(); ++p) {
                double d = nodeDistance(*options.metric, *node, **p);
                if (d < min) {
                    min = d;
                    minNode = p;
                }
            }
            return *minNode;
        }

    private:
        Node* nearestNode(Node* node) const
        {
            Node* minNode = 0;
            double min = 1e9;
            for (int i = 0; i < (int)cellsInUse.size(); i++) {
                const Cell& cell = *cellsInUse[i];
                for (NI p = cell.begin(); p != cell.end(); ++p) {
                    double d = nodeDistance(*options.metric, *node, **p);
                    if (d < min) {
                        min = d;
                        minNode = *p;
                    }
                }
            }
            RW_ASSERT(minNode);
            return minNode;
        }

    public:
        Node* nodeNearTo(Node* node)
        {
            switch (options.nearNodeSelection) {
            case SBLOptions::UniformSelect:
                return randomNodeUniform();
            case SBLOptions::UniformFromCell: {
                Cell& cell = cellOfNode(node);
                if (cell.empty())
                    return randomNodeUniform();
                else
                    return randomNodeFromCell(cell);
            }
            case SBLOptions::NearestFromCell: {
                Cell& cell = cellOfNode(node);
                if (cell.empty())
                    return randomNodeUniform();
                else
                    return nearestNodeFromCell(cell, node);
            }
            case SBLOptions::NearestNode:
                return nearestNode(node);
            }
            RW_ASSERT(0);
            return 0;
        }

    private:
        SpatialIndex(const SpatialIndex& other);
        SpatialIndex& operator=(const SpatialIndex& other);
    };
}

//----------------------------------------------------------------------
// The SBL graph

namespace
{
    enum TreeChoice { Start, Goal };

    class SBL
    {
        typedef std::vector<Node*> NodeVector;
        typedef NodeVector::const_iterator NI;

        SBLOptions options;

        // The nodes of both of the trees.
        NodeVector nodes;

        // A spatial index for each tree.
        SpatialIndex start_index;
        SpatialIndex goal_index;

        // Direct references to the start and goal nodes.
        NodeVector start_nodes;
        NodeVector goal_nodes;

        // Set once at reset().
        bool _isReset;

    public:
        SBL(const Q& from,
            const Q& to,
            const SBLOptions& options)
            :
            options(options),
            start_index(options),
            goal_index(options),
            _isReset(true)
        {
            RW_ASSERT(0 < options.connectRadius && options.connectRadius <= 1);

            RW_ASSERT(!inCollision(from));
            RW_ASSERT(!inCollision(to));

            start_nodes.push_back(newNode(from, 0));
            goal_nodes.push_back(newNode(to, 0));

            reset();
        }

    public:
        Node* addRootNode(const Q& q, const TreeChoice& choice)
        {
            Node* node = newNode(q, 0);
            addIndex(node, choice);
            rootNodesOf(choice).push_back(node);
            return node;
        }

    public:
        void reset()
        {
            rebuildSpatialIndexes(randomIndexPair((int)start_nodes.at(0)->q.size()));
            _isReset = true;
        }

    public:
        TreeChoice selectTree()
        {
            switch (options.treeSelection) {
            case SBLOptions::UniformTree:
                if (Random::ran(0, 1) < 0.5) return Start;
                else return Goal;
            case SBLOptions::WeightedTree: {
                const double sizeStart = indexOf(Start).size();
                const double sizeGoal = indexOf(Goal).size();

                const double total = sizeStart + sizeGoal;

                if (Random::ran(0, total) < sizeStart)
                    return Goal;
                else
                    return Start;
            }
            case SBLOptions::SmallestTree: {
                const double sizeStart = indexOf(Start).size();
                const double sizeGoal = indexOf(Goal).size();
                if (sizeStart < sizeGoal) return Start;
                else return Goal;
            }
            case SBLOptions::LargestTree: {
                const double sizeStart = indexOf(Start).size();
                const double sizeGoal = indexOf(Goal).size();
                if (sizeStart > sizeGoal) return Start;
                else return Goal;
            }}
            RW_ASSERT(0);
            return Start;
        }

    private:
        bool inCollision(const Q& q)
        {
            return options.constraint.getQConstraint().inCollision(q);
        }

    public:
        Node* expand(const TreeChoice& choice)
        {
            while (true) {
                Node* node = indexOf(choice).randomNode();
                RW_ASSERT(node);

                const Q q = options.expansion->expand(node->q);
                if (!q.empty()) {
                    Node* other = newNode(q, node);
                    addIndex(other, choice);
                    return other;
                }
            }
            RW_ASSERT(0);
            return 0;
        }

    private:
        TreeChoice opposite(const TreeChoice& choice)
        {
            switch (choice) {
            case Start: return Goal;
            case Goal: return Start;
            }

            RW_ASSERT(0);
            return Start;
        }

    private:
        SpatialIndex& indexOf(const TreeChoice& choice)
        {
            switch (choice) {
            case Start: return start_index;
            case Goal: return goal_index;
            }

            RW_ASSERT(0);
            return start_index;
        }

    private:
        NodeVector& rootNodesOf(const TreeChoice& choice)
        {
            switch (choice) {
            case Start: return start_nodes;
            case Goal: return goal_nodes;
            }

            RW_ASSERT(0);
            return start_nodes;
        }

    private:
        void addIndex(Node* node, const TreeChoice& choice)
        {
            indexOf(choice).addNode(node);
        }

    public:
        Motion connect(Node* node, const TreeChoice& choice)
        {
            if (options.connectAt == SBLOptions::ConnectAlways ||
                (_isReset && options.connectAt == SBLOptions::ConnectAtReset))
            {
                _isReset = false;

                Node* near2 = indexOf(opposite(choice)).nodeNearTo(node);
                RW_ASSERT(near2);

                const double dist = nodeDistance(*options.metric, *node, *near2);
                if (dist < options.connectRadius) {
                    return connectTrees(node, near2);
                }
            }
            return Motion();
        }

    private:
        void setNodeType(const NodeVector& roots, const NodeType& type)
        {
            for (NI p = roots.begin(); p != roots.end(); ++p) {
                RW_ASSERT(*p);
                (*p)->type = type;
            }
        }

    private:
        void rebuildSpatialIndexes(const IndexPair& pair)
        {
            for (NI p = nodes.begin(); p != nodes.end(); ++p) {
                RW_ASSERT(*p);
                (*p)->type = UnknownTree;
            }

            setNodeType(start_nodes, StartTree);
            setNodeType(goal_nodes, GoalTree);

            NodeVector start_index_nodes = start_nodes;
            NodeVector goal_index_nodes = goal_nodes;

            for (NI p = nodes.begin(); p != nodes.end(); ++p) {
                addNodeAndParents(*p, start_index_nodes, goal_index_nodes);
            }

            start_index.reset(start_index_nodes, pair);
            goal_index.reset(goal_index_nodes, pair);
        }

    private:
        void addNodeAndParents(
            Node* node,
            NodeVector& start_index_nodes,
            NodeVector& goal_index_nodes)
        {
            RW_ASSERT(node);
            if (node->type == UnknownTree) {
                RW_ASSERT(node->parent);

                addNodeAndParents(
                    node->parent, start_index_nodes, goal_index_nodes);

                RW_ASSERT(node->parent);

                node->type = node->parent->type;
                RW_ASSERT(node->type != UnknownTree);

                switch (node->parent->type) {
                case StartTree: start_index_nodes.push_back(node); break;
                case GoalTree: goal_index_nodes.push_back(node); break;
                default: RW_ASSERT(0);
                }
            }
        }

    private:
        Node* newNode(const Q& q, Node* parent)
        {
			Node* node = new Node(q, parent, options.constraint.getEdgeConstraint());
            nodes.push_back(node);
            return node;
        }

    private:
        void insertRootPath(Node* node, PriorityQueue& node_queue)
        {
            for (; node != 0; node = node->parent) {
                if (!node->edge->isFullyChecked())
                    node_queue.push(node);
            }
        }

    private:
        Motion connectTrees(Node* a, Node* b)
        {
            RW_ASSERT(a != b);

            // The bridge is an edge checker from the configuration of 'a' to 'b'.
            Node bridge(a->q, b, options.constraint.getEdgeConstraint());

            // Insert all relevant nodes in the priority queue.
            PriorityQueue node_queue;
            insertRootPath(&bridge, node_queue);
            insertRootPath(a, node_queue);

            // While the queue is non-empty, check edges for collisions.
            while (!node_queue.empty()) {
                Node* node = node_queue.top();
                node_queue.pop();

                if (node->edge->inCollisionPartialCheck()) {
                    if (node != &bridge) {
                        deleteEdge(a, b, node, bridge.edge);
                    }
                    return Motion();
                }

                if (!node->edge->isFullyChecked()) {
                    node_queue.push(node);
                }
            }

            // OK, we found a path.
            return extractSolutionPath(a, b);
        }

    private:
        bool elem(Node* node, const NodeVector& nodes)
        {
            return std::find(nodes.begin(), nodes.end(), node) != nodes.end();
        }

    private:
        Motion extractSolutionPath(Node* a, Node* b)
        {
            RW_ASSERT(a != b);

            Motion pa;
            Motion pb;

            Node* root_a = pushPathFromRoot(a, pa);
            Node* root_b = pushPathFromRoot(b, pb);

            const bool a_is_start_b_is_end =
                elem(root_a, start_nodes) && elem(root_b, goal_nodes);
            const bool b_is_start_a_is_end =
                elem(root_b, start_nodes) && elem(root_a, goal_nodes);

            if (a_is_start_b_is_end) {
                reverseOnto(pb, pa);
                return pa;
            } else if (b_is_start_a_is_end) {
                reverseOnto(pa, pb);
                return pb;
            } else {
                RW_ASSERT(0);
                return Motion();
            }
        }

    private:
        Node* pushPathFromRoot(Node* node, Motion& path)
        {
            Node* root = node;
            if (node->parent != 0)
                root = pushPathFromRoot(node->parent, path);

            path.push_back(node->q);
            return root;
        }

    private:
		void deleteEdge(Node* a, Node* b, Node* node, QEdgeConstraintIncremental::Ptr bridge)
        {
            RW_ASSERT(a != b);

            const bool ok =
                searchReverseEdges(b, a, node, bridge)
                || searchReverseEdges(a, b, node, bridge);

            // std::cout << "-- delete --\n";

            reset();

            RW_ASSERT(ok);
        }

    private:
        bool searchReverseEdges(
            Node* parent,
            Node* from,
            Node* to,
			QEdgeConstraintIncremental::Ptr edge)
        {
            RW_ASSERT(parent != from);

            // Root reached: The search failed.
            if (from == 0)
                return false;

            // The node has been found at this point.
            bool ok = from == to;

            // Optionally search downwards.
            ok = ok || searchReverseEdges(from, from->parent, to, from->edge);

            // Reverse the edge if the node was found.
            if (ok) {
                from->parent = parent;
                from->edge = edge;
            }
            return ok;
        }

    public:
        ~SBL()
        {
            for (NI p = nodes.begin(); p != nodes.end(); ++p)
                delete *p;
        }

    private:
        SBL(const SBL& other);
        SBL& operator=(const SBL& other);
    };
}

//----------------------------------------------------------------------
// The path planning function

namespace
{
    void addAsRootIfCfree(
        const Q& q,
        const QConstraint& constraint,
        SBL& sbl,
        const TreeChoice& tree)
    {
        if (!constraint.inCollision(q))
            sbl.addRootNode(q, tree);
    }

    void addAsRootsIfCfree(
        const Motion& qs,
        const QConstraint& constraint,
        SBL& sbl,
        const TreeChoice& tree)
    {
        BOOST_FOREACH(const Q& q, qs) {
            addAsRootIfCfree(q, constraint, sbl, tree);
        }
    }

    void addAsRootIfCfreeSample(
        QSampler& sampler,
        const QConstraint& constraint,
        SBL& sbl,
        const TreeChoice& tree)
    {
        const Q& q = sampler.sample();
        if (!q.empty()) return addAsRootIfCfree(q, constraint, sbl, tree);
    }

    void verifyStartGoalOrThrow(const Q& from, const Q& to)
    {
        if (from.empty() || to.empty() || from.size() != to.size())
            RW_THROW(
                "Bad dimensions of start/goal configurations for path planner:\n"
                << from
                << "\n"
                << to);
    }

    Q getRootConfiguration(
        const Q& q,
        QSampler& sampler,
        const StopCriteria& stop)
    {
        if (!q.empty()) return q;
        else {
            while (!stop.stop() && !sampler.empty()) {
                const Q q = sampler.sample();
                if (!q.empty())
                    return q;
            }
        }
        return Q();
    }
}

Motion NS::findConnection(
    const Q& from_raw,
    const Q& to_raw,
    const Motion& fromSamples,
    const Motion& toSamples,
    QSampler& fromSampler,
    QSampler& toSampler,
    const SBLOptions& options,
    const StopCriteria& stop)
{
    // Sanity check of dimensions to catch some errors and have some nicer error
    // messages.

    const Q from = getRootConfiguration(from_raw, fromSampler, stop);
    const Q to = getRootConfiguration(to_raw, toSampler, stop);
    if (from.empty() || to.empty()) return Motion();

    verifyStartGoalOrThrow(from, to);

    const int resetCount = options.resetCount;
    const int rootSampleInterval = options.rootSampleInterval;

    if (options.constraint.getQConstraint().inCollision(from) ||
        options.constraint.getQConstraint().inCollision(to))
    {

        return Motion();
    }

    SBL sbl(from, to, options);

    addAsRootsIfCfree(fromSamples, options.constraint.getQConstraint(), sbl, Start);
    addAsRootsIfCfree(toSamples, options.constraint.getQConstraint(), sbl, Goal);

    for (int count = 1;; count++) {
        if (stop.stop()) return Motion();

        const TreeChoice choice = sbl.selectTree();
        Node* node = sbl.expand(choice);
        Motion path = sbl.connect(node, choice);

        if (!path.empty()) return path;

        if (count % rootSampleInterval == 0) {
            addAsRootIfCfreeSample(fromSampler, options.constraint.getQConstraint(), sbl, Start);
            addAsRootIfCfreeSample(toSampler, options.constraint.getQConstraint(), sbl, Goal);
        }

        if (count % resetCount == 0) sbl.reset();
    }

    return Motion();
}

Motion NS::findPath(
    const Q& from,
    const Q& to,
    const SBLOptions& options,
    const StopCriteria& stop)
{
	QSampler::Ptr emptySampler = QSampler::makeEmpty();
    return findConnection(
        from,
        to,
        Motion(),
        Motion(),
        *emptySampler,
        *emptySampler,
        options,
        stop);
}

Motion NS::findApproach(
    const Q& from,
    const Q& to,
    const Motion& toSamples,
    QSampler& toSampler,
    const SBLOptions& options,
    const StopCriteria& stop)
{
	QSampler::Ptr emptySampler = QSampler::makeEmpty();
    return findConnection(
        from,
        to,
        Motion(),
        toSamples,
        *emptySampler,
        toSampler,
        options,
        stop);
}
