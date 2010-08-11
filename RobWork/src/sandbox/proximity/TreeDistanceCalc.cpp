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
        const CollisionModel *treeA, *treeB;
        bool isAfromA;
        int aIdx,bIdx;
        double dist;


        BVJob():
            treeA(NULL),treeB(NULL),isAfromA(true)
            {}

        BVJob(const CollisionModel *tA, const CollisionModel *tB, int ai, int bi, bool isAA = true):
            treeA(tA),treeB(tB),isAfromA(isAA),aIdx(ai),bIdx(bi)
            {
            }

        BVJob(const CollisionModel *tA, const CollisionModel *tB, bool isAA = true):
            treeA(tA),treeB(tB),isAfromA(isAA)
            {
            }
    };


    /**
     * @brief this only works for oriented bounding volumes
     */
    class BalancedTreeDistanceCalc: public TreeDistanceCalc {
    public:

        BalancedTreeDistanceCalc(BVDistanceCalc* bvdistcalc, int n=200):
            _bvDistCalc(bvdistcalc),_BVstack(n),_BVstackIdx(0)
        {
            initVars();
        }

        virtual ~BalancedCollider(){};

        double distance(
            const rw::math::Transform3D<>& fTA, const CollisionModel& cmA,
            const rw::math::Transform3D<>& fTB, const CollisionModel& cmB){
            double sdistance = 100000;
            initVars();

            const OBVTree *treeA = cmA.getBVTree();
            const OBVTree *treeB = cmB.getBVTree();

            RW_DEBUG("Collides...");

            const Transform3D<> tATtB = inverse(fTA) * fTB;
            const Transform3D<> tBTtA = inverse(tATtB);

            push( BVJob(&treeA,&treeB,0,0) );

            RW_DEBUG("Process children of root");
            while( !empty() ){
                RW_DEBUG("Get JOB: " << _BVstackIdx);
                BVJob job = *top();
                pop();

                RW_ASSERT(job.treeA!=NULL);
                RW_ASSERT(job.treeB!=NULL);

                // test if we can cull this job away
                if( job.dist>sdistance )
                    continue;

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
                RW_ASSERT( !parent.isLeaf() );

                RW_DEBUG("2: " );
                // now calculate the distance between A and the children of B
                const int nrOfChildren = parent.getNrOfChildren();
                std::vector<double> dists(nrOfChildren);
                for(int i=0; i<nrOfChildren; i++){
                    const OBBTree<> &child = *(parent.getChildren()[i]);
                    const OBB<> &cobbB = child.getOBB();

                    bool res;
                    if(job.isAfromA)
                        dists[i] = _bvdistcalc->calcDistance( cobbA, cobbB,ATtB*cobbB.getTransform());
                    else
                        dists[i] = _bvdistcalc->calcDistance( cobbB, cobbA, inverse(ATtB*cobbB.getTransform()));
                }
                // now add the jobs to the stack such that the closest bv's will be testet first.
                for(int i=0; i<nrOfChildren;i++){
                    // make sure we don't add extra unneccesary work
                    if(dist[i]>sdistance)
                        continue;
                    if( child.isLeaf() && job.treeA->isLeaf() ){
                        // test primitives against each other
                        _triDistCalc
                    } else if( job.treeA->isLeaf() ){
                        push( BVJob(job.treeA, &child, job.isAfromA) );
                    } else {
                        push( BVJob(&child, job.treeA, !job.isAfromA) );
                    }
                }


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
        BVDistanceCalc *_bvdistcalc;
        TriDistanceCalc *_tridistcalc;

        std::vector< BVJob > _BVstack;

        bool _firstContact;
        int _BVstackIdx, _nrOfBVTests, _nrOfCollidingBVs;
        int _nrOfPrimTests, _nrOfCollidingPrims;

    };

}

TreeCollider* TreeDistanceCalc::makeBalancedDFSDistanceCalc(BVDistanceCalc* bvdistcalc){
    std::cout << "making bvalanced dfs collider!" << std::endl;
    return new BalancedCollider(bvcollider);
}

/*
TreeCollider* TreeCollider::makeBalancedBFSCollider(){

}
*/
