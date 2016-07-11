#ifndef RWSIM_SIMULATOR_CONTACTGRAPH_HPP_
#define RWSIM_SIMULATOR_CONTACTGRAPH_HPP_

#include <vector>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include "ConstraintEdge.hpp"
#include "ConstraintNode.hpp"
#include "CNodePairMap.hpp"

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace simulator {
	class ContactModelFactory;
	class CNodePool;

    /**
     * @brief creates a graph where nodes can be physical, logical and
     * compound entities and the edges between nodes are constraints
     * of some sort.
     *
     */
    class ContactGraph
    {
    public:

        /**
         * @brief Constructor - constructs a ContactGraph with the
         * given nodes and edges. Ownership of nodes and edges is taken
         * @param nodes [in] all initial nodes in the graph
         * @param edges [in] all initial constraints in the graph
         * @param factory [in] a factory for finding collision constraints
         */
    	ContactGraph(CNodePool *pool, ContactModelFactory &factory);

    	/**
    	 * @brief Destructor
    	 */
    	virtual ~ContactGraph();

    	/**
    	 * @brief updates overlab information. updates, inserts and removes edges.
    	 * Also applies logical and coherence testing.
    	 */
    	void broadPhase(rw::kinematics::State &state,
    	                bool shortCircuit);

    	/**
    	 * @brief apply narrow phase collision detection, and update
    	 * edge states. If shortCircuit is set to true then narrowPhase will
    	 * stop when penetration is detected.
    	 */
    	bool narrowPhase(rw::kinematics::State &state,
    					 bool shortCircuit);

    	/**
    	 * @brief calculates contact information from cached broadphase
    	 * and narrowphase results.
    	 */
    	void updateContacts(rw::kinematics::State &state);


    	void applyLogicalCoherenceTest(bool shortCircuit);

    	/**
    	 * @brief Roll state back to previous state
    	 * @param state
    	 */
    	void rollBack(rw::kinematics::State &state);

    	/**
    	 * @brief save current state
    	 */
    	void saveState();

    	/**
    	 * @brief all nodes that are connected to \b n through a
    	 * ConstraintEdge::Structural edge are returned
    	 * @param n
    	 * @return
    	 */
    	std::vector< ConstraintNode* > getStaticConnectedNodes(ConstraintNode* n);

    	std::vector< ConstraintNode* > getConnectedNodes(ConstraintNode* n, ConstraintEdge::EdgeType type);

    	std::vector< std::vector<ConstraintEdge*> > computeGroups();

    	std::vector< std::vector<ConstraintEdge*> > getPhysicalGroups();

    	void writeToFile(std::string filename);

   	void resetState(rw::kinematics::State &state){
    		_oFrames.clear();
    		_fEdges.clear();
    	}

    private:

    	void narrowPhaseCalc(ConstraintEdge& edge, rw::kinematics::State &state);

        void traverseGroup(ConstraintEdge &edge, std::vector<ConstraintEdge*>& group);

    	void remove(ConstraintNode *node);

    private:
        // maps containing edges and nodes
    	//std::map<rw::kinematics::Frame*, ConstraintNode*> _frameToNode;
    	//std::map< NodePair, ConstraintEdge* > _nodeToEdge;
       	ContactModelFactory &_factory;
    	CNodePool *_pool;
    	CNodePairMap<ConstraintEdge*> _pairToEdge;
    	rw::kinematics::FrameMap<ConstraintNode*> _frameToNode;

    	//std::vector<ConstraintNode*> _nodes;
    	rw::kinematics::FramePairSet _oFrames;

    public:
    	std::vector<ConstraintEdge*> _fEdges; // filtered edges

    };

} // namespace dynamics
}

#endif /*CONTACTGRAPH_HPP_*/
