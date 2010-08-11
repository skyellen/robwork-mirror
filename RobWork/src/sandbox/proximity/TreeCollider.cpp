/*
 * TreeCollider.cpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#include "TreeCollider.hpp"

#include <rw/math/Transform3D.hpp>

#include "BVCollider.hpp"

using namespace rw::math;

//#define RW_DEBUG( str ) std::cout << str  << std::endl;
#define TIMING( str, func ) \
    { long start = rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }

//#define TIMING( str, func ) {func;}


namespace {

    struct BVJob {
    public:
        const OBBTree<> *treeA, *treeB;
        bool isAfromA;
        int aIdx,bIdx;


        BVJob():
            treeA(NULL),treeB(NULL),isAfromA(true)
            {}

        BVJob(const OBBTree<> *tA, const OBBTree<> *tB, int ai, int bi, bool isAA = true):
            treeA(tA),treeB(tB),isAfromA(isAA),aIdx(ai),bIdx(bi)
            {
            }

        BVJob(const OBBTree<> *tA, const OBBTree<> *tB, bool isAA = true):
            treeA(tA),treeB(tB),isAfromA(isAA)
            {
            }
    };

    class BalancedCollider: public TreeCollider {

    public:

        BalancedCollider(BVCollider* bvcollider, int n=200):
            _bvCollider(bvcollider),_BVstack(n),_BVstackIdx(0)
        {
            initVars();
        }

        virtual ~BalancedCollider(){};

        bool collides(
            const rw::math::Transform3D<>& fTA, const OBBTree<>& treeA,
            const rw::math::Transform3D<>& fTB, const OBBTree<>& treeB){
            bool incollision = false;
            initVars();


            RW_DEBUG("Collides...");

            const Transform3D<> tATtB = inverse(fTA) * fTB;
            const Transform3D<> tBTtA = inverse(tATtB);
            {
                // test
                const OBB<> &obbA = treeA.getOBB();
                const OBB<> &obbB = treeB.getOBB();

                const Transform3D<> nATtB = inverse(obbA.getTransform())*tATtB;

                _nrOfBVTests = 1;
                _nrOfCollidingBVs = 0;

                if( !_bvCollider->inCollision(obbA, obbB, nATtB*obbB.getTransform()) ){
                    RW_DEBUG("ROOT DOES NOT COLLIDE");
                    return false;
                }
                RW_DEBUG("ROOT COLLIDES");
                _nrOfCollidingBVs++;
                RW_DEBUG("Create JOB");
                push( BVJob(&treeA,&treeB,0,0) );
            }

            RW_DEBUG("Process children of root");
            while( !empty() ){
                RW_DEBUG("Get JOB: " << _BVstackIdx);
                BVJob job = *top();
                pop();
                if( job.treeA==NULL || job.treeB==NULL )
                    RW_THROW("THEY ARE NULL");
                RW_DEBUG("after Get JOB: " << _BVstackIdx);
                const OBB<> &cobbA = job.treeA->getOBB();
                Transform3D<> ATtB;
                if( job.isAfromA ){
                    ATtB = inverse( cobbA.getTransform() )*tATtB;
                } else {
                    ATtB = inverse( cobbA.getTransform() )*tBTtA;
                }
                RW_DEBUG("1: " );
                const OBBTree<> &parent = *(job.treeB);
                if( parent.isLeaf() )
                    RW_THROW("SHOULD NOT BE POSSIBLLE!");
                RW_DEBUG("2: " );
                // test if job.treeA collides with any of treeB's children
                const int nrOfChildren = parent.getNrOfChildren();
                for(int i=0; i<nrOfChildren; i++){
                    RW_DEBUG("1: "<< i);
                    const OBBTree<> &child = *(parent.getChildren()[i]);

                    const OBB<> &cobbB = child.getOBB();

                    bool res;
                    if(job.isAfromA)
                        res = _bvCollider->inCollision( cobbA, cobbB,ATtB*cobbB.getTransform());
                    else
                        res = _bvCollider->inCollision( cobbB, cobbA, inverse(ATtB*cobbB.getTransform()));

                    _nrOfBVTests++;
                    if( !res )
                        continue;

                    _nrOfCollidingBVs++;

                    if( child.isLeaf() && job.treeA->isLeaf() ){
                        // test primitives against each other
                        incollision |= true;
                        if( _firstContact && incollision)
                            return true;
                        continue;
                    } else if( job.treeA->isLeaf() ){
                        push( BVJob(job.treeA, &child, job.isAfromA) );
                    } else {
                        push( BVJob(&child, job.treeA, !job.isAfromA) );
                    }
                }
            }
            return incollision;
        }

        BVJob* top(){
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

        void push( const BVJob& job){

            if( _BVstackIdx+1>=(int)_BVstack.size() ){
                _BVstack.resize(_BVstackIdx*2);
            }
            _BVstack[_BVstackIdx] = job;
            _BVstackIdx++;
        }

        int size(){
            return _BVstackIdx;
        }

        BVJob& getJob(int idx){
            if( idx>=(int)_BVstack.size() ){
                _BVstack.resize(idx*2);
            }
            return _BVstack[idx];
        }

        int getMemUsage(){
            return _BVstackIdx*sizeof(BVJob);
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
        BVCollider *_bvCollider;
        std::vector< BVJob > _BVstack;

        bool _firstContact;
        int _BVstackIdx, _nrOfBVTests, _nrOfCollidingBVs;
        int _nrOfPrimTests, _nrOfCollidingPrims;

    };

}

TreeCollider* TreeCollider::makeBalancedDFSCollider(BVCollider* bvcollider){
    std::cout << "making bvalanced dfs collider!" << std::endl;
    return new BalancedCollider(bvcollider);
}

/*
TreeCollider* TreeCollider::makeBalancedBFSCollider(){

}
*/
