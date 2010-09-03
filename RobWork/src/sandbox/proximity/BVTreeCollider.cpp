/*
 * TreeCollider.cpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#include "BVTreeCollider.hpp"

#include <rw/math/Transform3D.hpp>

#include "OBBCollider.hpp"

using namespace rw::proximity;
using namespace rw::math;
using namespace rw::geometry;

//#define RW_DEBUG( str ) std::cout << str  << std::endl;
#define TIMING( str, func ) \
    { long start = rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }

//#define TIMING( str, func ) {func;}


namespace {

	template<class NODE>
    struct BVJob {
    public:
        NODE nodeA, nodeB;
        int aIdx,bIdx;

        BVJob()
        {}

        BVJob(NODE tA, NODE tB):
            nodeA(tA),nodeB(tB)
        {}

        BVJob(NODE tA, NODE tB, int ai, int bi):
        	nodeA(tA),nodeB(tB),aIdx(ai),bIdx(bi)
        {}
    };

    /**
     * @brief this tree collider is used for oriented bounding volumes.
     */
    template<class BVCOLLIDER>
    class BalancedCollider: public BVTreeCollider {
    public:

    	typedef typename BVTREE::iterator TreeNode;
    	typedef typename BVTREE::BVType BV;
    	typedef BVJob< TreeNode > Job;

        BalancedCollider(OBBCollider<>* bvcollider, int n=200):
            _bvCollider(bvcollider),_BVstack(n),_BVstackIdx(0)
        {
            initVars();
        }

        virtual ~BalancedCollider(){};

        bool collides(
            const rw::math::Transform3D<>& fTA, const BVTree& treeA,
            const rw::math::Transform3D<>& fTB, const BVTree& treeB){
            bool incollision = false;

            initVars();

            TreeNode nodeA = treeA<BVTREE::Node>.getIterator();
            TreeNode nodeB = treeB.getIterator();

            RW_DEBUG("Collides...");
            const Transform3D<> tATtB = inverse(fTA) * fTB;
            const Transform3D<> tBTtA = inverse(tATtB);
            _nrOfBVTests = 0;
            _nrOfCollidingBVs = 0;

            push( Job(nodeA,nodeB,0,0) );

            RW_DEBUG("Process children of root");
            while( !empty() ){
                RW_DEBUG("Get JOB: " << _BVstackIdx);
                Job job = *top();
                pop();

                if( job.nodeA.isLeaf() && job.nodeB.isLeaf() ){
                	// Collide primitives
					incollision |= true;
					if( _firstContact && incollision)
						return true;
                	continue;
                }

                RW_DEBUG("after Get JOB: " << _BVstackIdx);
                const BV &cbvA = job.nodeA.getBV();
                const BV &cbvB = job.nodeB.getBV();

                Transform3D<> ATtB = inverse( cbvA.getTransform() )*tATtB;
                if( _bvCollider->inCollision( cbvA, cbvB, ATtB*cbvB.getTransform()) ){
                	_nrOfCollidingBVs++;
                	// push back new jobs, handle if one of the bounding volumes are leaf nodes
                	bool descentA = true;
                	if(descentA){
                		push( Job(job.nodeA.right(), job.nodeB) );
                		push( Job(job.nodeA.left(), job.nodeB) );
                	} else {
                		push( Job(job.nodeA, job.nodeB.right()) );
                		push( Job(job.nodeA, job.nodeB.left()) );
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
            _firstContact = false;
        }

        BVCOLLIDER *_bvCollider;
        std::vector< Job > _BVstack;

        bool _firstContact;
        int _BVstackIdx, _nrOfBVTests, _nrOfCollidingBVs;
        int _nrOfPrimTests, _nrOfCollidingPrims;

    };

}
#include "BinaryBVTree.hpp"
TreeCollider<BinaryBVTree<OBB<> > >* TreeColliderFactory::makeBalancedDFSColliderOBB(OBBCollider<>* bvcollider){
    //return new BalancedCollider<BVTree<OBB<> > >(bvcollider);
	return new BalancedCollider<BinaryBVTree<OBB<> >, OBBCollider<> >(bvcollider);
}


/*
TreeCollider* TreeCollider::makeBalancedBFSCollider(){

}
*/
