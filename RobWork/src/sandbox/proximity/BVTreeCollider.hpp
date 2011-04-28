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
#include <sandbox/geometry/TRIDeviller.hpp>
#include <boost/foreach.hpp>

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

        typedef rw::common::Ptr<BVTreeCollider<BVTREE> > Ptr;


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
	        typedef typename BV::value_type T;

	        OBVTreeDFSCollider(BVCOLLIDER* bvcollider, DESCENTSTRATEGY* descendStrat, int n=200):
	            _bvCollider(bvcollider),_descendStrat(descendStrat),_BVstack(n),_BVstackIdx(0),_firstContact(true)
	        {
	            _primCollider = new rw::geometry::TRIDeviller<T>();
	            initVars();
	        }

	        virtual ~OBVTreeDFSCollider(){
	            delete _primCollider;
	        };

	        bool collides(
	            const rw::math::Transform3D<typename BV::value_type>& fTA, const BVTREE& treeA,
	            const rw::math::Transform3D<typename BV::value_type>& fTB, const BVTREE& treeB){

	            using namespace rw::math;
	            using namespace rw::geometry;

	            bool incollision = false;
	            //std::cout << fTA << std::endl;
	            //std::cout << fTB << std::endl;
	            initVars();

	            typename DESCENTSTRATEGY::State descendState;
	            typename BVTREE::node_iterator nodeA = treeA.getRootIterator();
	            typename BVTREE::node_iterator nodeB = treeB.getRootIterator();

	            RW_DEBUG("Collides...");
	            Transform3D<typename BV::value_type> tATtB;
	            Transform3D<typename BV::value_type>::invMult(fTA, fTB, tATtB);
	            //const Transform3D<typename BV::value_type> tBTtA = inverse(tATtB);
                //Transform3D<typename BV::value_type> tATtB = inverse(fTA)*fTB;

	            _nrOfBVTests = 0;
	            _nrOfCollidingBVs = 0;
	            //std::cout << tATtB << std::endl;

	            push( Job(nodeA,nodeB,descendState) );

	            RW_DEBUG("Process children of root");
	            while( !empty() ){
	                RW_DEBUG("Get JOB: " << _BVstackIdx);
	                Job job = *top();
	                pop();
                    //std::cout << "\nJ" << _BVstackIdx;

	                RW_DEBUG("after Get JOB: " << _BVstackIdx);
	                const BV &cbvA = job.nodeA.getBV();
	                const BV &cbvB = job.nodeB.getBV();

	                Transform3D<typename BV::value_type> aATtB;
	                Transform3D<typename BV::value_type>::invMult(cbvA.getTransform(), tATtB, aATtB);
	                //Transform3D<typename BV::value_type> aATtB = inverse(cbvA.getTransform()) * tATtB;
	                if( _bvCollider->inCollision( cbvA, cbvB, aATtB*cbvB.getTransform()) ){
	                    //std::cout << aATtB*cbvB.getTransform() << "\n";
	                    //std::cout << cbvA.getTransform() << std::endl;
	                    //std::cout << cbvB.getTransform() << std::endl;
                        //std::cout << job.nodeA.depth() << " -- " << cbvA.getHalfLengths() << std::endl;
	                    //std::cout << job.nodeB.depth() << " -- " << cbvB.getHalfLengths() << std::endl;

	                    _nrOfCollidingBVs++;
	                    if( job.nodeA.isLeaf() && job.nodeB.isLeaf() ){
	                        //std::cout << "COLLISION" << std::endl;
	                        // TODO: Collide primitives
	                        BOOST_FOREACH(const Triangle<T>& tria, job.nodeA.getPrimitives()){
	                            BOOST_FOREACH(const Triangle<T>& trib, job.nodeB.getPrimitives()){
	                                if( _primCollider->inCollision(tria, trib, tATtB) ){
	                                    incollision = true;
	                                    // add triangle indicies to result

	                                    if(_firstContact)
	                                        return true;
	                                }
	                            }
	                        }
	                        //incollision = true;
	                        //_primitiveCollider->inCollision(job.nodeA, job.nodeB, tATtB);



	                        //if( _firstContact && incollision)
	                        //    return true;
	                        continue;
	                    }

	                    // push back new jobs, handle if one of the bounding volumes are leaf nodes
	                    bool descentA = _descendStrat->descentIntoA(cbvA, cbvB, job._state );
	                    if( (descentA && !job.nodeA.isLeaf()) || job.nodeB.isLeaf() ){
	                        // TODO: optimize such that only 1 is pushed on stack, the other is kept in local variables
	                        if( job.nodeA.hasRight() ){
                                push();
                                top()->nodeA = job.nodeA.right();
                                top()->nodeB = job.nodeB;
                                top()->_state = job._state;
	                        }

                            if( job.nodeA.hasLeft() ){
                                push();
                                top()->nodeA = job.nodeA.left();
                                top()->nodeB = job.nodeB;
                                top()->_state = job._state;
                            }
	                    } else {
	                        //push( Job(job.nodeA, job.nodeB.right(), job._state) );
	                        //push( Job(job.nodeA, job.nodeB.left(), job._state) );
	                        if( job.nodeB.hasRight() ){
                                push();
                                top()->nodeA = job.nodeA;
                                top()->nodeB = job.nodeB.right();
                                top()->_state = job._state;
	                        }

	                        if( job.nodeB.hasLeft() ){
                                push();
                                top()->nodeA = job.nodeA;
                                top()->nodeB = job.nodeB.left();
                                top()->_state = job._state;
	                        }
	                    }
	                }
	                _nrOfBVTests++;
	            }
                //std::cout << tATtB << std::endl;
	            //std::cout << "_nrOfBVTests: " << _nrOfBVTests << "  _nrOfCollidingBVs:" << _nrOfCollidingBVs << " incollision:" << incollision << std::endl;
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
	        rw::geometry::TRIDeviller<T> *_primCollider;
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
