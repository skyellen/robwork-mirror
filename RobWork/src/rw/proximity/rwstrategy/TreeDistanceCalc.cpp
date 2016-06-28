/*
 * TreeCollider.cpp
 *
 *  Created on: 28-10-2008
 *      Author: jimali
 */

#include "TreeDistanceCalc.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/TriDistanceCalc.hpp>

using namespace rw::math;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::common;

//#define RW_DEBUG( str ) std::cout << str  << std::endl;
#define TIMING( str, func ) \
    { long start = rw::common::TimerUtil::currentTimeMs(); \
    func; \
     long end = rw::common::TimerUtil::currentTimeMs(); \
    std::cout << str <<":" << (end-start) <<"ms"<<std::endl;  }

//#define TIMING( str, func ) {func;}


namespace {



    /**
     * @brief this only works for oriented bounding volumes
     */
    class BalancedTreeDistanceCalc: public TreeDistanceCalc {
    private:
        struct BVJob;

    public:
        typedef BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator NodeIterator;
        //typedef ::rw::Traits<BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> > >::NodeIterator NodeIterator;
        typedef ::rw::Traits<TreeDistanceCalc::BVTree>::BVType BVType;
        typedef ::rw::Traits<TreeDistanceCalc::BVTree>::PRIMType PRIMType;
        typedef ::rw::Traits<BVType>::value_type value_type;
        typedef ::rw::Traits<BVType>::value_type T;



        BalancedTreeDistanceCalc(rw::common::Ptr<TreeDistanceCalc::BVDISTANCECALC> bvdistcalc, int n=200):
            _bvdistcalc(bvdistcalc),_BVstack(n),_BVstackIdx(0)
        {
            initVars();
        }

        virtual ~BalancedTreeDistanceCalc(){};

        double distance(
            const rw::math::Transform3D<>& fTA, const TreeDistanceCalc::BVTree& cmA,
            const rw::math::Transform3D<>& fTB, const TreeDistanceCalc::BVTree& cmB)
        {
			RW_THROW("Not implemented correctly");
            using namespace rw::geometry;
            using namespace rw::math;


            double sdistance = 100000;
            initVars();

            NodeIterator nodeA = cmA.getRootIterator();
            NodeIterator nodeB = cmB.getRootIterator();

            Transform3D<value_type> tATtB;
            Transform3D<value_type>::invMult(fTA, fTB, tATtB);

            push( BVJob(nodeA,nodeB,0,0) );

            RW_DEBUG("Process children of root");
            while( !empty() ){
                RW_DEBUG("Get JOB: " << _BVstackIdx);
                BVJob job = *top();
                pop();


                // test if we can cull this job away
                if( job.dist>sdistance )
                    continue;

                const OBB<> &cobbA = job.treeA.bv();
                Transform3D<> ATtB;
                if( job.isAfromA ){
                    ATtB = inverse( cobbA.getTransform() )*tATtB;
                } else {
                    ATtB = inverse( cobbA.getTransform() )*inverse(tATtB);
                }


                RW_ASSERT( !job.treeB.isLeaf() );

                // now calculate the distance between A and the children of B
                const OBB<> &cobbBl = job.treeB.left().bv();
                double distLeft = _bvdistcalc->distance( cobbA, cobbBl, ATtB*cobbBl.getTransform());
                const OBB<> &cobbBr = job.treeB.right().bv();
                double distRight = _bvdistcalc->distance( cobbA, cobbBr, ATtB*cobbBr.getTransform());

                // we push jobs on the stack such that the one with whartest distance is picked first
                if( (distLeft < distRight) ){
                    push( BVJob(job.treeB.right(), job.treeA, !job.isAfromA) );
                    push( BVJob(job.treeB.left(), job.treeA, !job.isAfromA) );
                } else {
                    push( BVJob(job.treeB.left(), job.treeA, !job.isAfromA) );
                    push( BVJob(job.treeB.right(), job.treeA, !job.isAfromA) );
                }
            }
            // TODO: implementation is not working properly
            return sdistance;
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
        rw::common::Ptr<TreeDistanceCalc::BVDISTANCECALC > _bvdistcalc;
        rw::common::Ptr<rw::geometry::TriDistanceCalc<T> > _tridistcalc;


    private:
        struct BVJob {
        public:
            BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator treeA, treeB;

            // state which will be changed at some point
            bool isAfromA;
            int aIdx,bIdx;
            double dist;

            BVJob():
                isAfromA(true)
                {}

            BVJob(const BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator& tA,
                  const BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator &tB,
                  int ai, int bi, bool isAA = true):
                treeA(tA),treeB(tB),isAfromA(isAA),aIdx(ai),bIdx(bi)
                {
                }

            BVJob(const BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator& tA,
                  const BinaryBVTree<rw::geometry::OBB<>, rw::geometry::Triangle<> >::NodeIterator &tB, bool isAA = true):
                treeA(tA),treeB(tB),isAfromA(isAA)
                {
                }
        };




        std::vector< BVJob > _BVstack;

        bool _firstContact;
        int _BVstackIdx, _nrOfBVTests, _nrOfCollidingBVs;
        int _nrOfPrimTests, _nrOfCollidingPrims;

    };

}

rw::common::Ptr<TreeDistanceCalc> TreeDistanceCalc::makeBalancedDFSDistanceCalc(rw::common::Ptr< TreeDistanceCalc::BVDISTANCECALC > bvdistcalc){
    std::cout << "making bvalanced distance calculator!" << std::endl;
    return rw::common::ownedPtr( new BalancedTreeDistanceCalc(bvdistcalc) );
}

/*
TreeCollider* TreeCollider::makeBalancedBFSCollider(){

}
*/
