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

#ifndef RW_PROXIMITY_BVTREETOLERANCECOLLIDER_HPP_
#define RW_PROXIMITY_BVTREETOLERANCECOLLIDER_HPP_

//#include <rw/geometry/OBB.hpp>
//#include <rw/geometry/BVCollider.hpp>
//#include <rw/geometry/TriTriIntersectDeviller.hpp>

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

namespace rw {
namespace proximity {

	/**
	 * @brief this class encapsulates the methods for iterating through
	 * two hierachical OBV trees while testing if the OBV's are disjoint by
	 * more than a specified tolerance.
	 *
	 *
	 * @note The TreeCollider is statefull and should therefore NOT be used by
	 * multiple threads. The state is small so there is only little to no overhead
	 * cloning the TreeCollider.
	 */
    template<class BVTREE>
	class BVTreeToleranceCollider {
	private:

	public:
        //! @brief smart pointer for this class
        typedef rw::common::Ptr<BVTreeToleranceCollider<BVTREE> > Ptr;

        /**
         * @brief destructor
         */
		virtual ~BVTreeToleranceCollider(){};

		/**
		 * @brief tests if two BV trees are colliding.
		 * @param fTA [in] transform from reference frame \b f to tree \b treeA root.
		 * @param treeA [in]
		 * @param fTB [in] transform from reference frame \b f to tree \b treeB root.
		 * @param treeB [in]
		 */
		virtual bool collides(
			const rw::math::Transform3D<typename BVTREE::value_type>& fTA, const BVTREE& treeA,
			const rw::math::Transform3D<typename BVTREE::value_type>& fTB, const BVTREE& treeB,
			double tolerance,
			std::vector<std::pair<int,int> > *collidingPrimitives = NULL) = 0;

		/**
		 * @brief set the query type
		 */
		virtual void setQueryType(CollisionStrategy::QueryType type){ _queryType=type;};

        /**
         * @brief get the collision query type
         */
		virtual CollisionStrategy::QueryType getQueryType(){ return _queryType;};

		//! type of the primitive in collision callb ack function
		//typedef boost::function<void(int,int)> PrimitivesInCollisionCB;
		//virtual void setPrimitivesInCollisionCB(PrimitivesInCollisionCB cb){ _pCB = cb;}

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
	protected:
		CollisionStrategy::QueryType _queryType;
		//PrimitivesInCollisionCB _pCB;
	};

#ifdef this_is_old_stuff

    /**
     * @brief factory for generating tolerance collider strategies
     */
    class BVTreeToleranceColliderFactory {
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
        static BVTreeToleranceCollider<BVTREE>* makeBalancedDFSToleranceColliderOBB(){
            rw::geometry::OBBCollider<typename BVTREE::value_type>* bvcollider = new rw::geometry::OBBCollider<typename BVTREE::value_type>();
            BalancedDescentStrategy<BVTREE>* dstrategy = new BalancedDescentStrategy<BVTREE>();
            return makeDFSCollider<rw::geometry::OBBCollider<typename BVTREE::value_type>, BalancedDescentStrategy<BVTREE>, BVTREE>(bvcollider, dstrategy);
        }

        /**
         * @brief creates a depth first search tree collider
         * @param bvcollider
         * @param dstrat
         * @return
         */
        template<class COLLIDER, class DESCENT_STRATEGY, class BVTREE>
        static BVTreeToleranceCollider<BVTREE>* makeDFSCollider(COLLIDER* bvcollider, DESCENT_STRATEGY* dstrat){
            return new OBVTreeDFSCollider<BVTREE, COLLIDER, DESCENT_STRATEGY>(bvcollider, dstrat);
        }


        static BVTreeToleranceCollider<BinaryOBBPtrTreeD>* makeOBBPtrTreeBDFSColliderD();

        static BVTreeToleranceCollider<BinaryOBBPtrTreeF>* makeOBBPtrTreeBDFSColliderF();


    private:
        /**
         * @brief this tree collider is used for oriented bounding volumes.
         */
        template<class BVTREE, class BVCOLLIDER, class DESCENTSTRATEGY>
        class OBVTreeDFSToleranceCollider: public BVTreeToleranceCollider<BVTREE> {
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

            OBVTreeDFSToleranceCollider(BVCOLLIDER* bvcollider, DESCENTSTRATEGY* descendStrat, int n=200):
                _bvCollider(bvcollider),_descendStrat(descendStrat),_BVstack(n),_BVstackIdx(0),_firstContact(true)
            {
                _primCollider = new rw::geometry::TriTriIntersectDeviller<T>();
                initVars();
            }

            virtual ~OBVTreeDFSToleranceCollider(){
                delete _primCollider;
            };

            bool collides(
                const rw::math::Transform3D<typename BV::value_type>& fTA, const BVTREE& treeA,
                const rw::math::Transform3D<typename BV::value_type>& fTB, const BVTREE& treeB,
                std::vector<std::pair<int,int> > *collidingPrimitives=NULL){

                using namespace rw::math;
                using namespace rw::geometry;

                bool incollision = false;
                //std::cout << fTA << std::endl;
                //std::cout << fTB << std::endl;
                initVars();

                typename DESCENTSTRATEGY::State descendState;
                typename BVTREE::node_iterator nodeA = treeA.getRootIterator();
                typename BVTREE::node_iterator nodeB = treeB.getRootIterator();

                //RW_DEBUG("Collides...");
                Transform3D<typename BV::value_type> tATtB;
                Transform3D<typename BV::value_type>::invMult(fTA, fTB, tATtB);
                //const Transform3D<typename BV::value_type> tBTtA = inverse(tATtB);

                //Transform3D<typename BV::value_type> tATtB = inverse(fTA)*fTB;

                _nrOfBVTests = 0;
                _nrOfCollidingBVs = 0;
                //std::cout << tATtB << std::endl;

                push( Job(nodeA,nodeB,descendState) );
                int nodeIdx = 0;
               //std::cout << "graph CollisionTree {\n";
                //std::cout << "{\n 1 -- 2 -- 3 -- 4 -- 5 -- 6 -- 7 -- 8 -- 9 -- 10 ;\n }\n"
                //RW_DEBUG("Process children of root");
                while( !empty() ){
                    //RW_DEBUG("Get JOB: " << _BVstackIdx);
                    Job job = *top();
                    //std::cout << "\t" << job.nodeA.getId() <<" [style=filled,color=red, rank=" << (int)job.nodeA.depth() << "]; \n";
                    //std::cout << "\t" << job.nodeB.getId() <<" [style=filled,color=blue, rank=" << (int)job.nodeB.depth() << "]; \n";
                    //std::cout << "\t" << job.nodeA.getId() << " -- " << job.nodeB.getId() <<" [style=dotted]; \n";

                    pop();
                    //std::cout << "\nJ" << _BVstackIdx;

                    //RW_DEBUG("after Get JOB: " << _BVstackIdx);
                    const BV &cbvA = job.nodeA.getBV();
                    const BV &cbvB = job.nodeB.getBV();



                    Transform3D<typename BV::value_type> aATtB;
                    Transform3D<typename BV::value_type>::invMult(cbvA.getTransform(), tATtB, aATtB);
                    //Transform3D<typename BV::value_type> aATtB = inverse(cbvA.getTransform()) * tATtB;
                    bool incol = _bvCollider->inCollision( cbvA, cbvB, aATtB*cbvB.getTransform());
                    nodeIdx++;

                    if( incol ){

                        if( job.nodeA.isLeaf() && job.nodeB.isLeaf() ){
                           //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" [style=filled,color=green, label= \"" << nodeIdx << "\"] \n";
                            const size_t nrTrisA = treeA.getNrTriangles(job.nodeA);
                            const size_t nrTrisB = treeB.getNrTriangles(job.nodeB);
                            Triangle<T> tria, trib;

                            for(size_t ai=0;ai<nrTrisA;ai++){
                                int triaidx = treeA.getTriangle(job.nodeA,tria,ai);
                                for(size_t bi=0;bi<nrTrisB;bi++){
                                    int tribidx = treeB.getTriangle(job.nodeB,trib,bi);
                                    _nrOfPrimTests++;
                                    if( _primCollider->inCollision(tria, trib, tATtB) ){
                                        incollision = true;
                                        //if(collidingPrimitives)
                                        //    collidingPrimitives->push_back( std::make_pair(triaidx, tribidx) );

                                        // add triangle indicies to result
                                        if(_firstContact)
                                            return true;
                                    }
                                }
                            }
                            continue;
                        }

                        //std::cout << aATtB*cbvB.getTransform() << "\n";
                        //std::cout << cbvA.getTransform() << std::endl;
                        //std::cout << cbvB.getTransform() << std::endl;
                        //std::cout << job.nodeA.depth() << " -- " << cbvA.getHalfLengths() << std::endl;
                        //std::cout << job.nodeB.depth() << " -- " << cbvB.getHalfLengths() << std::endl;

                        _nrOfCollidingBVs++;


                        // push back new jobs, handle if one of the bounding volumes are leaf nodes
                        bool descentA = _descendStrat->descentIntoA(cbvA, cbvB, job._state );
                        if( (descentA && !job.nodeA.isLeaf()) || job.nodeB.isLeaf() ){
                            // TODO: optimize such that only 1 is pushed on stack, the other is kept in local variables
                            if( job.nodeA.hasRight() ){
                                push();
                                //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" -- "
                                //          << job.nodeA.right().getId() << "" << job.nodeB.getId() << "\n";

                                //std::cout << "\t" << job.nodeA.getId() << " -- " << job.nodeA.right().getId() <<";\n";
                                top()->nodeA = job.nodeA.right();
                                top()->nodeB = job.nodeB;
                                top()->_state = job._state;
                            }

                            if( job.nodeA.hasLeft() ){
                                //std::cout << "\t" << job.nodeA.getId() << " -- " << job.nodeA.left().getId() <<";\n";
                                //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" -- "
                                //          << job.nodeA.left().getId() << "" << job.nodeB.getId() << "\n";

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
                                //std::cout << "\t" << job.nodeB.getId() << " -- " << job.nodeB.right().getId() <<";\n";
                                //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" -- "
                                //          << job.nodeA.getId() << "" << job.nodeB.right().getId() << "\n";

                                top()->nodeA = job.nodeA;
                                top()->nodeB = job.nodeB.right();
                                top()->_state = job._state;
                            }

                            if( job.nodeB.hasLeft() ){
                                push();
                                //std::cout << "\t" << job.nodeB.getId() << " -- " << job.nodeB.left().getId() <<";\n";
                                //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" -- "
                                //          << job.nodeA.getId() << "" << job.nodeB.left().getId() << "\n";

                                top()->nodeA = job.nodeA;
                                top()->nodeB = job.nodeB.left();
                                top()->_state = job._state;
                            }
                        }
                    }

                    _nrOfBVTests++;
                }
                //std::cout << " }\n";
                //std::cout << tATtB << std::endl;
                //std::cerr << "_nrOfBVTests: " << _nrOfBVTests
                //          << "  _nrOfCollidingBVs:" << _nrOfCollidingBVs
                //          << " incollision:" << incollision
                //          << " _nrOfPrimTests: " << _nrOfPrimTests << std::endl;
                return incollision;
            }

            Job* top(){
                if(_BVstackIdx==0)
                    return NULL;
                return &_BVstack[_BVstackIdx-1];
            }
            void pop(){
               // RW_ASSERT(_BVstackIdx!=0);
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
                _firstContact = BVTreeCollider<BVTREE>::_queryType==CollisionStrategy::FirstContact;
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
#endif

}
}


#endif /* TREECOLLIDER_HPP_ */
