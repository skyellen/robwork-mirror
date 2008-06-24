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

#include "SBLInternal.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>

#include "tnt_array2d.h"

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

    double log2(double val)
    {
        static double scale = 1 / log((double)2);
        return scale * log(val);
    }
}

//----------------------------------------------------------------------
// Distance metrics and other functions for normalized configurations.

namespace
{
    bool inBounds(const Q& q)
    {
        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            if (q[i] < 0 || 1 < q[i]) {
                return false;
            }
        }
        return true;
    }

    // A randomly selected configuration from the part of the box of radius
    // 'radius' that overlaps with the configuration space.
    Q randomBoxQ(const Q& q, double radius)
    {
        Q result(q.size());

        const int len = (int)q.size();
        for (int i = 0; i < len; i++) {
            double lower = std::max(0.0, q[i] - radius);
            double upper = std::min(1.0, q[i] + radius);
            result[i] = Math::ran(lower, upper);
        }
        return result;
    }
}

//----------------------------------------------------------------------
// The accuracy with which edges are checked for collisions.

namespace
{
    const double stepSize = 0.01;
}

//----------------------------------------------------------------------
// Segments

namespace
{
    // A 'Segment' represents an undirected connection between two
    // configurations.
    class Segment
    {
        Q from;
        Q to;
        Q diff;

        double length_diff;
        int collision_checking_level;
        int final_collision_checking_level;

        bool remove; // This variable is for sanity checking only.

    public:
        Segment(const Q& from, const Q& to)
            : from(from),
              to(to),
              diff(to - from),
              length_diff(MetricUtil::normInf(diff)),
              collision_checking_level(0),
              remove(false)
        {
            if (length_diff < stepSize) { // If they are close:
                final_collision_checking_level = 0;
            } else {
                final_collision_checking_level =
                    (int)ceil(log2(length_diff / stepSize));
            }
        }

        bool upgradeSegment(const QConstraint& constraint)
        {
            RW_ASSERT(
                collision_checking_level < final_collision_checking_level
                && !remove);

            const int k = ++collision_checking_level;
            const double scale = pow(2.0, -(double)k);
            const int end = (int)floor(pow(2.0, k - 1));

            for (int i = 0; i < end; i++) {
                const Q pos = from + (scale * (1 + 2 * i)) * diff;
                if (constraint.inCollision(pos)) {
                    remove = true;
                    return false;
                }
            }
            return true;
        }

        bool isSafe() const
        {
            RW_ASSERT(!remove);
            return collision_checking_level == final_collision_checking_level;
        }

        double priority() const
        {
            return length_diff * pow(2.0, -collision_checking_level);
        }
    };
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
        Segment segment;

    public:
        NodeType type;

    public:
        Node(const Q& q, Node* parent)
            :
            q(q),
            parent(parent),
            segment(q, parent ? parent->q : q),
            type(UnknownTree)
        {}

    private:
        Node(const Node& other);
        Node& operator=(const Node& other);
    };

    double nodeDistance(const Node& na, const Node& nb)
    {
        return MetricUtil::distInf(na.q, nb.q);
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
            return a->segment.priority() < b->segment.priority();
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
            Math::ranI(0, dim),
            Math::ranI(0, dim));
    }

    class SpatialIndex
    {
        int mappingSize;
        int nodeCount;

        typedef std::vector<Node*> Cell;

        typedef TNT::Array2D<Cell> ArrayMapping;
        ArrayMapping arrayMap;

        typedef std::map<IndexPair, Cell> TreeMapping;
        TreeMapping treeMap;

        std::pair<double, double> xrange;
        std::pair<double, double> yrange;

        int xindex;
        int yindex;

        std::vector<Cell*> cellsInUse;

        // Whether to use the array or the search tree mapping technique.
        bool useArrayMap;

        // The average number of nodes to insert in each cell.
        double nodesPerCell;

        // How to select the node to connect to.
        SBLOptions::NearNodeSelection nearNodeSelection;

    public:
        SpatialIndex(const SBLOptions& options)
            : mappingSize(1),
              nodeCount(0),
              arrayMap(mappingSize, mappingSize),
              useArrayMap(options.useArrayMap),
              nodesPerCell(options.nodesPerCell),
              nearNodeSelection(options.nearNodeSelection)
        {
            arrayMap = Cell();

            // No good default values exist here.
            xindex = -1;
            yindex = -1;
            xrange = std::make_pair(0, 1);
            yrange = std::make_pair(0, 1);
        }

    private:
        int arraySize(int tree_size) const
        {
            return (int)floor(sqrt((double)tree_size / nodesPerCell));
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

            const int newSize = arraySize((int)nodes.size());
            mappingSize = std::max(newSize, mappingSize);

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
            if (useArrayMap) return;

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
            if (useArrayMap) clearArrayMap();
            else clearTreeMap();

        }

    private:
        void clearTreeMap()
        {
            treeMap.clear();
        }

    private:
        void clearArrayMap()
        {
            if (arrayMap.dim1() < mappingSize) {
                arrayMap = ArrayMapping(mappingSize, mappingSize);
            }
            arrayMap = Cell();
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

            if (useArrayMap) {

                RW_ASSERT(0 <= xindex && xindex < (int)node->q.size());
                RW_ASSERT(0 <= yindex && yindex < (int)node->q.size());

                double xd = node->q[xindex];
                double yd = node->q[yindex];

                // But by using (mappingSize - 1) we are not using the cells at
                // this index?
                int x = (int)floor(xd * (mappingSize - 1));
                int y = (int)floor(yd * (mappingSize - 1));

                RW_ASSERT(0 <= x && x < mappingSize);
                RW_ASSERT(0 <= y && y < mappingSize);

                return std::make_pair(x, y);

            } else {

                double vx = node->q[xindex];
                double vy = node->q[yindex];

                double xlow = xrange.first;
                double xhigh = xrange.second;
                double ylow = yrange.first;
                double yhigh = yrange.second;

                // But why should this index pair be particularly good or any
                // better than the above? A serious timing experiment is needed
                // to justify this type of code.
                int x = (int)floor(mappingSize * vx / (xhigh - xlow));
                int y = (int)floor(mappingSize * vy / (yhigh - ylow));

                return std::make_pair(x, y);
            }
        }

    private:
        Cell& cellOfNode(Node* node)
        {
            return getCell(indexPairOf(node));
        }

    private:
        Cell& getCell(const IndexPair& pair)
        {
            if (useArrayMap) {
                int x = pair.first;
                int y = pair.second;

                RW_ASSERT(0 <= x && x < mappingSize && 0 <= y && y < mappingSize);
                return arrayMap[x][y];
            } else {
                return treeMap[pair];
            }
        }

    private:
        static Node* randomNodeFromCell(const Cell& cell)
        {
            RW_ASSERT(!cell.empty());
            return
                cell.at(
                    Math::ranI(0, (int)cell.size()));
        }

    private:
        Node* randomNodeStandard() const
        {
            return
                randomNodeFromCell(
                    *cellsInUse.at(
                        Math::ranI(0, (int)cellsInUse.size())));
        }

    private:
        Node* randomNodeSkewed() const
        {
            // With more elaborate data structures this could be done in
            // logarithmic time. In practice however the number of cells is
            // typically small.

            const double pos = Math::ran(0, 1);
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
            int choice = Math::ranI(0, nodeCount);

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
            double min = nodeDistance(*node, **p);
            NI minNode = p;

            for (++p; p != cell.end(); ++p) {
                double d = nodeDistance(*node, **p);
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
                    double d = nodeDistance(*node, **p);
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
            switch (nearNodeSelection) {
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

        // The collision constraint for which the motion planning is done.
        const QConstraint& constraint;

        // The nodes of both of the trees.
        NodeVector nodes;

        // A spatial index for each tree.
        SpatialIndex start_index;
        SpatialIndex goal_index;

        // Direct references to the start and goal nodes.
        NodeVector start_nodes;
        NodeVector goal_nodes;

        // Various options.
        double extendRadius;
        double connectRadius;
        SBLOptions::ExpandMode expandMode;

    public:
        SBL(const Q& from,
            const Q& to,
            const QConstraint& constraint,
            const SBLOptions& options)
            : constraint(constraint),
              start_index(options),
              goal_index(options),
              extendRadius(options.extendRadius),
              connectRadius(options.connectRadius),
              expandMode(options.expandMode)
        {
            RW_ASSERT(0 < connectRadius && connectRadius <= 1);
            RW_ASSERT(0 < extendRadius && extendRadius <= 1);

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
            rebuildSpatialIndexes(
                randomIndexPair(start_nodes.at(0)->q.size()));
        }

    public:
        TreeChoice selectTree()
        {
            if (Math::ran(0, 1) < 0.5)
                return Start;
            else
                return Goal;
        }

    private:
        bool inCollision(const Q& q)
        {
            RW_ASSERT(inBounds(q));
            return constraint.inCollision(q);
        }

    public:
        Node* expand(const TreeChoice& choice)
        {
            const double radius = extendRadius;

            while (true) {

                Node* node = indexOf(choice).randomNode();
                RW_ASSERT(node);

                for (double n = 1; n < 5; n++) {
                    const Q q =
                        expandQ(
                            node->q,
                            radius / n,
                            expandMode);

                    if (!inCollision(q)) {
                        Node* other = newNode(q, node);
                        addIndex(other, choice);
                        return other;
                    }
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
            Node* near = indexOf(opposite(choice)).nodeNearTo(node);
            RW_ASSERT(near);

            const double dist = nodeDistance(*node, *near);
            if (dist < connectRadius) {
                return connectTrees(node, near);
            }
            return Motion();
        }

    private:
        static Q expandQ(
            const Q& q,
            double radius,
            SBLOptions::ExpandMode mode)
        {
            switch (mode) {
            case SBLOptions::UniformBox:
                return randomBoxQ(q, radius);
            }
            RW_ASSERT(0);
            return q;
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
            Node* node = new Node(q, parent);
            nodes.push_back(node);
            return node;
        }

    private:
        void insertRootPath(Node* node, PriorityQueue& node_queue)
        {
            for (; node != 0; node = node->parent) {
                if (!node->segment.isSafe())
                    node_queue.push(node);
            }
        }

    private:
        Motion connectTrees(Node* a, Node* b)
        {
            RW_ASSERT(a != b);

            // The bridge is a segment from the configuration of 'a' to 'b'.
            Node bridge(a->q, b);

            // Insert all relevant nodes in the priority queue.
            PriorityQueue node_queue;
            insertRootPath(&bridge, node_queue);
            insertRootPath(a, node_queue);

            // While the queue is non-empty, check segments for collisions.
            while (!node_queue.empty()) {
                Node* node = node_queue.top();
                node_queue.pop();

                if (!node->segment.upgradeSegment(constraint)) {
                    if (node != &bridge) {
                        deleteEdge(a, b, node, bridge.segment);
                    }
                    return Motion();
                }

                if (!node->segment.isSafe())
                    node_queue.push(node);
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
        void deleteEdge(Node* a, Node* b, Node* node, const Segment& bridge)
        {
            RW_ASSERT(a != b);

            const bool ok =
                searchReverseEdges(b, a, node, bridge)
                || searchReverseEdges(a, b, node, bridge);

            reset();

            RW_ASSERT(ok);
        }

    private:
        bool searchReverseEdges(
            Node* parent,
            Node* from,
            Node* to,
            const Segment& segment)
        {
            RW_ASSERT(parent != from);

            // Root reached: The search failed.
            if (from == 0)
                return false;

            // The node has been found at this point.
            bool ok = from == to;

            // Optionally search downwards.
            ok = ok || searchReverseEdges(from, from->parent, to, from->segment);

            // Reverse the edge if the node was found.
            if (ok) {
                from->parent = parent;
                from->segment = segment;
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
}

Motion NS::findConnection(
    const Q& from,
    const Q& to,
    const Motion& fromSamples,
    const Motion& toSamples,
    QSampler& fromSampler,
    QSampler& toSampler,
    const QConstraint& constraint,
    const SBLOptions& options,
    StopCriteriaPtr stop)
{
    // Sanity check of dimensions to catch some errors and have some nicer error
    // messages.
    verifyStartGoalOrThrow(from, to);

    const int resetCount = options.resetCount;
    const int rootSampleInterval = options.rootSampleInterval;

    if (constraint.inCollision(from) || constraint.inCollision(to)) return Motion();

    SBL sbl(from, to, constraint, options);

    addAsRootsIfCfree(fromSamples, constraint, sbl, Start);
    addAsRootsIfCfree(toSamples, constraint, sbl, Goal);

    for (int count = 1;; count++) {
        if (stop->stop()) return Motion();

        const TreeChoice choice = sbl.selectTree();
        Node* node = sbl.expand(choice);
        Motion path = sbl.connect(node, choice);

        if (!path.empty()) return path;

        if (count % rootSampleInterval == 0) {
            addAsRootIfCfreeSample(fromSampler, constraint, sbl, Start);
            addAsRootIfCfreeSample(toSampler, constraint, sbl, Goal);
        }

        if (count % resetCount == 0) sbl.reset();
    }

    return Motion();
}

Motion NS::findPath(
    const Q& from,
    const Q& to,
    QConstraint& constraint,
    const SBLOptions& options,
    StopCriteriaPtr stop)
{
    std::auto_ptr<QSampler> emptySampler = QSampler::makeEmpty();
    return findConnection(
        from,
        to,
        Motion(),
        Motion(),
        *emptySampler,
        *emptySampler,
        constraint,
        options,
        stop);
}

Motion NS::findApproach(
    const Q& from,
    const Q& to,
    const Motion& toSamples,
    QSampler& toSampler,
    QConstraint& constraint,
    const SBLOptions& options,
    StopCriteriaPtr stop)
{
    std::auto_ptr<QSampler> emptySampler = QSampler::makeEmpty();
    return findConnection(
        from,
        to,
        Motion(),
        toSamples,
        *emptySampler,
        toSampler,
        constraint,
        options,
        stop);
}
