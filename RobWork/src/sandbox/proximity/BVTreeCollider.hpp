/*
 * TreeCollider.hpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#ifndef RW_PROXIMITY_BVTREECOLLIDER_HPP_
#define RW_PROXIMITY_BVTREECOLLIDER_HPP_

#include <sandbox/geometry/OBB.hpp>
#include "BVTree.hpp"
#include "BinaryBVTree.hpp"
#include "BVCollider.hpp"
#include <sandbox/geometry/OBB.hpp>
#include "OBBCollider.hpp"

namespace rw {
namespace proximity {

	/**
	 * @brief this class encapsulates the methods for iterating through
	 * two hierachical OBV trees while testing if the BV's are disjoint.
	 */
    template<class BVTREE>
	class BVTreeCollider {
	private:

	public:
		virtual ~BVTreeCollider(){};

		/**
		 *
		 * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
		 * @param treeA [in]
		 * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
		 * @param treeB [in]
		 */
		virtual bool collides(
			const rw::math::Transform3D<typename BVTREE::value_type>& fTA, const BVTREE& treeA,
			const rw::math::Transform3D<typename BVTREE::value_type>& fTB, const BVTREE& treeB) = 0;

		/**
		 * @brief returns the amount of heap memmory used by the tree collider.
		 * @return nr of bytes used
		 */
		virtual int getMemUsage() = 0;

		virtual int getNrOfTestedBVs(){ return -1;};
		virtual int getNrOfCollidingBVs(){ return -1;};

		virtual int getNrOfTestedPrimitives(){ return -1;};
		virtual int getNrOfCollidingPrimitives(){ return -1;};



		//TreeCollider* makeBalancedBFSCollider();

		/**
		 * @brief creates a tree collider that performes depth first search
		 * of bv collisions between two hierarchical trees. The search is
		 * performed using a synchronous descent strategy which means that the
		 * traversal descents both trees at the same time.
		 */
		//static TreeCollider* makeSyncDFSCollider(BVCollider* bvcollider);

		/**
		 *
		 * @param weight
		 * @return
		 */
		//TreeCollider* makeWeightedDFSCollider(const BVWeight& weight);

		//TreeCollider* makeWeightedBFSCollider(const BVWeight& weight);

	};


	class BVTreeColliderFactory {
	public:

		template<class DERIVED, class DESCENTSTATE, class BVTREE>
		struct BVDescentStrategy {
			typedef typename BVTREE::Node BVNODE;
			typedef DESCENTSTATE State;

			bool descentIntoA(const BVNODE& bvA, const BVNODE& bvB, DESCENTSTATE& state){
				return static_cast<DERIVED*>(this)->descentIntoA(bvA,bvB, state);
			}

		};

        struct BalancedDescentState {
		    BalancedDescentState():previousRes(true){};
            bool previousRes;
        };

		template<class BVTREE>
		struct BalancedDescentStrategy: public BVDescentStrategy<BalancedDescentStrategy<BVTREE>,BalancedDescentState,BVTREE>{
			typedef typename BVTREE::Node BVNODE;
			typedef BalancedDescentState State;
			BalancedDescentStrategy(){}
			bool descentIntoA(const BVNODE& bvA, const BVNODE& bvB, BalancedDescentState& state){
				state.previousRes = !state.previousRes;
				return state.previousRes;
			}
		};

		/**
		 * @brief creates a tree collider that performes depth first search
		 * of bv collisions between two hierarchical trees. The search is
		 * balanced in the way that it equally switches between descending in the
		 * trees.
		 */
		template<class BVTREE>
		static BVTreeCollider<BVTREE>* makeBalancedDFSColliderOBB(){
		    OBBCollider<typename BVTREE::value_type>* bvcollider = new OBBCollider<typename BVTREE::value_type>();
			BalancedDescentStrategy<BVTREE>* dstrategy = new BalancedDescentStrategy<BVTREE>();
			return makeDFSCollider<OBBCollider<typename BVTREE::value_type>, BalancedDescentStrategy<BVTREE>, BVTREE>(bvcollider, dstrategy);
		}

		/**
		 * @brief creates a depth first search tree collider
		 * @param bvcollider
		 * @param dstrat
		 * @return
		 */
		template<class COLLIDER, class DESCENT_STRATEGY, class BVTREE>
		static BVTreeCollider<BVTREE>* makeDFSCollider(COLLIDER* bvcollider, DESCENT_STRATEGY* dstrat){
		    return new OBVTreeDFSCollider<BVTREE, COLLIDER, DESCENT_STRATEGY>(bvcollider, dstrat);
		}


		static BVTreeCollider<BinaryOBBPtrTreeD>* makeOBBPtrTreeBDFSColliderD();
		static BVTreeCollider<BinaryOBBPtrTreeF>* makeOBBPtrTreeBDFSColliderF();


	private:


	    /**
	     * @brief this tree collider is used for oriented bounding volumes.
	     */
	    template<class BVTREE, class BVCOLLIDER, class DESCENTSTRATEGY>
	    class OBVTreeDFSCollider: public BVTreeCollider<BVTREE> {
	    public:

	        struct Job {
	        public:
	            typename BVTREE::node_iterator nodeA, nodeB;
	            typename DESCENTSTRATEGY::State _state;
	            //int aIdx,bIdx;

	            Job()
	            {}

	            Job(typename BVTREE::node_iterator tA,
	                typename BVTREE::node_iterator tB,
	                const typename DESCENTSTRATEGY::State& state):
	                nodeA(tA),nodeB(tB),_state(state)
	            {}
/*
	            BVJob(BVTREE::node_iterator tA, BVTREE::node_iterator tB, int ai, int bi):
	                nodeA(tA),nodeB(tB),aIdx(ai),bIdx(bi)
	            {}
	            */
	        };

	        typedef typename BVTREE::iterator TreeNode;
	        typedef typename BVTREE::BVType BV;

	        OBVTreeDFSCollider(BVCOLLIDER* bvcollider, DESCENTSTRATEGY* descendStrat, int n=200):
	            _bvCollider(bvcollider),_descendStrat(descendStrat),_BVstack(n),_BVstackIdx(0),_firstContact(true)
	        {
	            initVars();
	        }

	        virtual ~OBVTreeDFSCollider(){};

	        bool collides(
	            const rw::math::Transform3D<typename BV::value_type>& fTA, const BVTREE& treeA,
	            const rw::math::Transform3D<typename BV::value_type>& fTB, const BVTREE& treeB){

	            using namespace rw::math;

	            bool incollision = false;

	            initVars();

	            typename DESCENTSTRATEGY::State descendState;
	            typename BVTREE::node_iterator nodeA = treeA.getRootIterator();
	            typename BVTREE::node_iterator nodeB = treeB.getRootIterator();

	            RW_DEBUG("Collides...");
	            Transform3D<typename BV::value_type> tATtB;
	            Transform3D<typename BV::value_type>::invMult(fTA, fTB, tATtB);
	            const Transform3D<typename BV::value_type> tBTtA = inverse(tATtB);
	            _nrOfBVTests = 0;
	            _nrOfCollidingBVs = 0;


	            push( Job(nodeA,nodeB,descendState) );

	            RW_DEBUG("Process children of root");
	            while( !empty() ){
	                RW_DEBUG("Get JOB: " << _BVstackIdx);
	                Job job = *top();
	                pop();

	                if( job.nodeA.isLeaf() && job.nodeB.isLeaf() ){
	                    // Collide primitives
	                    incollision = true;
	                    if( _firstContact && incollision)
	                        return true;
	                    continue;
	                }

	                RW_DEBUG("after Get JOB: " << _BVstackIdx);
	                const BV &cbvA = job.nodeA.getBV();
	                const BV &cbvB = job.nodeB.getBV();

	                Transform3D<typename BV::value_type> ATtB;
	                Transform3D<typename BV::value_type>::invMult(cbvA.getTransform(), tATtB, ATtB);

	                if( _bvCollider->inCollision( cbvA, cbvB, ATtB*cbvB.getTransform()) ){
	                    _nrOfCollidingBVs++;
	                    // push back new jobs, handle if one of the bounding volumes are leaf nodes
	                    bool descentA = _descendStrat->descentIntoA(cbvA, cbvB, job._state );
	                    if( (descentA && !job.nodeA.isLeaf()) || job.nodeB.isLeaf() ){
	                        // TODO: optimize such that only 1 is pushed on stack, the other is kept in local variables
	                        //push( Job(job.nodeA.right(), job.nodeB, job._state) );
	                        //push( Job(job.nodeA.left(), job.nodeB, job._state) );
	                        push();
	                        top()->nodeA = job.nodeA.right();
                            top()->nodeB = job.nodeB;
                            top()->_state = job._state;

                            push();
                            top()->nodeA = job.nodeA.left();
                            top()->nodeB = job.nodeB;
                            top()->_state = job._state;
	                    } else {
	                        //push( Job(job.nodeA, job.nodeB.right(), job._state) );
	                        //push( Job(job.nodeA, job.nodeB.left(), job._state) );

                            push();
                            top()->nodeA = job.nodeA;
                            top()->nodeB = job.nodeB.right();
                            top()->_state = job._state;
                            push();
                            top()->nodeA = job.nodeA;
                            top()->nodeB = job.nodeB.left();
                            top()->_state = job._state;

	                    }
	                }
	                _nrOfBVTests++;
	            }
	            return incollision;
	        }

	        Job* top(){
	            if(_BVstackIdx==0)
	                return NULL;
	            return &_BVstack[_BVstackIdx-1];
	        }
	        void pop(){
	            _BVstackIdx--;
	        }
	        bool empty(){
	            return _BVstackIdx==0;
	        }
	        void push(){
                if( _BVstackIdx+1>=(int)_BVstack.size() ){
                    _BVstack.resize(_BVstackIdx*2);
                }
                _BVstackIdx++;
	        }

	        void push( const Job& job){

	            if( _BVstackIdx+1>=(int)_BVstack.size() ){
	                _BVstack.resize(_BVstackIdx*2);
	            }
	            _BVstack[_BVstackIdx] = job;
	            _BVstackIdx++;
	        }

	        int size(){
	            return _BVstackIdx;
	        }

	        Job& getJob(int idx){
	            if( idx>=(int)_BVstack.size() ){
	                _BVstack.resize(idx*2);
	            }
	            return _BVstack[idx];
	        }

	        int getMemUsage(){
	            return _BVstackIdx*sizeof(Job);
	        }

	        virtual int getNrOfTestedBVs(){ return _nrOfBVTests; };
	        virtual int getNrOfCollidingBVs(){ return _nrOfCollidingBVs;};

	        virtual int getNrOfTestedPrimitives(){ return _nrOfPrimTests;};
	        virtual int getNrOfCollidingPrimitives(){ return _nrOfCollidingPrims;};


	        void initVars(){
	            _BVstackIdx = 0;
	            _nrOfBVTests = 0;
	            _nrOfCollidingBVs = 0;
	            _nrOfPrimTests = 0;
	            _nrOfCollidingPrims = 0;
	            _firstContact = true;
	        }


	        BVCOLLIDER *_bvCollider;
	        DESCENTSTRATEGY *_descendStrat;
	        std::vector< Job > _BVstack;
	        int _BVstackIdx;
            bool _firstContact;

	        int _nrOfBVTests, _nrOfCollidingBVs;
	        int _nrOfPrimTests, _nrOfCollidingPrims;

	    };




	};

}
}


#endif /* TREECOLLIDER_HPP_ */
