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

#ifndef RW_PROXIMITY_OBVTREECOLLIDERDFS_HPP_
#define RW_PROXIMITY_OBVTREECOLLIDERDFS_HPP_

#include "BVTree.hpp"
#include "BVTreeCollider.hpp"

#include <rw/math/Transform3D.hpp>

namespace rw {
namespace proximity {


    /**
     * @brief this tree collider is used for oriented bounding volumes. The collider
     * traverse the trees in a depth first manner. The DESCENTSTRATEGY is used to choose
     * which node the traversal should pick first. The collider only works on trees of
     * the same type. The BVCOLLIDER and PRIMCOLLIDER is used to check is bounding volumes
     * or primitives are colliding.
     */
    template<class BVTREE, class BVCOLLIDER, class DESCENTSTRATEGY, class PRIMCOLLIDER>
    class OBVTreeDFSCollider: public BVTreeCollider<BVTREE> {
    private:
        struct Job;
    public:
        typedef typename Traits<BVTREE>::NodeIterator NodeIterator;
        typedef typename Traits<BVTREE>::BVType BVType;
        typedef typename Traits<BVType>::value_type value_type;
        typedef typename Traits<BVType>::value_type T;

        OBVTreeDFSCollider(BVCOLLIDER* bvcollider, PRIMCOLLIDER* primcollider, DESCENTSTRATEGY* descendStrat, int n=200):
            _bvCollider(bvcollider),_primCollider(primcollider),_descendStrat(descendStrat),_BVstack(n),_BVstackIdx(0),_firstContact(true)
        {
            initVars();
        }

        //! @brief detructor
        virtual ~OBVTreeDFSCollider(){
            delete _primCollider;
            delete _bvCollider;
            delete _descendStrat;
        };

        //! check if two trees are colliding
        bool collides(
            const rw::math::Transform3D<value_type>& fTA, const BVTREE& treeA,
            const rw::math::Transform3D<value_type>& fTB, const BVTREE& treeB,
            std::vector<std::pair<int,int> > *collidingPrimitives=NULL);

        int size(){ return _BVstackIdx; }

        int getMemUsage(){ return _BVstackIdx*sizeof(Job); }
        virtual int getNrOfTestedBVs(){ return _nrOfBVTests; };
        virtual int getNrOfCollidingBVs(){ return _nrOfCollidingBVs;};
        virtual int getNrOfTestedPrimitives(){ return _nrOfPrimTests;};
        virtual int getNrOfCollidingPrimitives(){ return _nrOfCollidingPrims;};

    private:
        inline Job* top(){
            if(_BVstackIdx==0)
                return NULL;
            return &_BVstack[_BVstackIdx-1];
        }

        inline void pop(){
           // RW_ASSERT(_BVstackIdx!=0);
            _BVstackIdx--;
        }

        inline bool empty() const{
            return _BVstackIdx==0;
        }
        inline void push(){
            if( _BVstackIdx+1>=(int)_BVstack.size() ){
                _BVstack.resize(_BVstackIdx*2);
            }
            _BVstackIdx++;
        }

        inline void push( const Job& job){

            if( _BVstackIdx+1>=(int)_BVstack.size() ){
                _BVstack.resize(_BVstackIdx*2);
            }
            _BVstack[_BVstackIdx] = job;
            _BVstackIdx++;
        }



        inline Job& getJob(int idx){
            if( idx>=(int)_BVstack.size() ){
                _BVstack.resize(idx*2);
            }
            return _BVstack[idx];
        }

        inline void initVars(){
            _BVstackIdx = 0;
            _nrOfBVTests = 0;
            _nrOfCollidingBVs = 0;
            _nrOfPrimTests = 0;
            _nrOfCollidingPrims = 0;
            _firstContact = BVTreeCollider<BVTREE>::_queryType==CollisionStrategy::FirstContact;
        }

    private:
        struct Job {
        public:
            NodeIterator nodeA, nodeB;
            typename Traits<DESCENTSTRATEGY>::StateType _state;

            Job()
            {}

            Job(NodeIterator tA,
                NodeIterator tB,
                const typename Traits<DESCENTSTRATEGY>::StateType& state):
                nodeA(tA),nodeB(tB),_state(state)
            {}
        };


        BVCOLLIDER *_bvCollider;
        PRIMCOLLIDER *_primCollider;
        DESCENTSTRATEGY *_descendStrat;
        std::vector< Job > _BVstack;
        int _BVstackIdx;
        bool _firstContact;

        int _nrOfBVTests, _nrOfCollidingBVs;
        int _nrOfPrimTests, _nrOfCollidingPrims;
    };


    // implementation of collides
    template<class BVTREE, class BVCOLLIDER, class DESCENTSTRATEGY, class PRIMCOLLIDER>
    bool OBVTreeDFSCollider<BVTREE,BVCOLLIDER,DESCENTSTRATEGY,PRIMCOLLIDER>::collides(
            const rw::math::Transform3D<typename Traits<typename Traits<BVTREE>::BVType>::value_type>& fTA, const BVTREE& treeA,
            const rw::math::Transform3D<typename Traits<typename Traits<BVTREE>::BVType>::value_type>& fTB, const BVTREE& treeB,
            std::vector<std::pair<int,int> > *collidingPrimitives)
    {
        typedef typename Traits<BVTREE>::NodeIterator NodeIterator;
        typedef typename Traits<BVTREE>::BVType BVType;
        typedef typename Traits<BVTREE>::PRIMType PRIMType;
        typedef typename Traits<BVType>::value_type value_type;
        // typedef typename Traits<BVType>::value_type T;

        using namespace rw::math;
        using namespace rw::geometry;

        bool incollision = false;
        //std::cout << fTA << std::endl;
        //std::cout << fTB << std::endl;
        initVars();

        typename Traits<DESCENTSTRATEGY>::StateType descendState = typename Traits<DESCENTSTRATEGY>::StateType ();
        NodeIterator nodeA = treeA.getRootIterator();
        NodeIterator nodeB = treeB.getRootIterator();

        //RW_DEBUG("Collides...");
        Transform3D<value_type> tATtB;
        Transform3D<value_type>::invMult(fTA, fTB, tATtB);
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
            pop();
            //std::cout << "\t" << job.nodeA.getId() <<" [style=filled,color=red, rank=" << (int)job.nodeA.depth() << "]; \n";
            //std::cout << "\t" << job.nodeB.getId() <<" [style=filled,color=blue, rank=" << (int)job.nodeB.depth() << "]; \n";
            //std::cout << "\t" << job.nodeA.getId() << " -- " << job.nodeB.getId() <<" [style=dotted]; \n";

            //std::cout << "\nJ" << _BVstackIdx;

            //RW_DEBUG("after Get JOB: " << _BVstackIdx);
            const BVType &cbvA = job.nodeA.getBV();
            const BVType &cbvB = job.nodeB.getBV();

            Transform3D<value_type> aATtB;
            Transform3D<value_type>::invMult(cbvA.getTransform(), tATtB, aATtB);
            //Transform3D<typename BV::value_type> aATtB = inverse(cbvA.getTransform()) * tATtB;
            bool incol = _bvCollider->inCollision( cbvA, cbvB, aATtB*cbvB.getTransform());
            nodeIdx++;
            _nrOfBVTests++;

            if( !incol ){
                continue;
            }

            const bool isALeaf = job.nodeA.isLeaf();
            const bool isBLeaf = job.nodeB.isLeaf();

            if( isALeaf && isBLeaf ){
               //std::cout << "\t" << job.nodeA.getId() << "" << job.nodeB.getId() <<" [style=filled,color=green, label= \"" << nodeIdx << "\"] \n";
                const size_t nrTrisA = treeA.getNrPrimitives(job.nodeA);
                const size_t nrTrisB = treeB.getNrPrimitives(job.nodeB);
                PRIMType tria, trib;

                for(size_t ai=0;ai<nrTrisA;ai++){
                    int triaidx = treeA.getPrimitive(job.nodeA,tria,ai);
                    for(size_t bi=0;bi<nrTrisB;bi++){
                        int tribidx = treeB.getPrimitive(job.nodeB,trib,bi);
                        _nrOfPrimTests++;
                        if( _primCollider->inCollision(tria, trib, tATtB) ){
                            incollision = true;
                            if(collidingPrimitives)
                                collidingPrimitives->push_back( std::make_pair(triaidx, tribidx) );

                            // add triangle indicies to result
                            if(_firstContact)
                                return true;
                        }
                    }
                }

            } else {
                _nrOfCollidingBVs++;

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
        }
        //std::cout << " }\n";
        //std::cout << tATtB << std::endl;
        //std::cerr << "_nrOfBVTests: " << _nrOfBVTests
        //          << "  _nrOfCollidingBVs:" << _nrOfCollidingBVs
        //          << " incollision:" << incollision
        //          << " _nrOfPrimTests: " << _nrOfPrimTests << std::endl;
        return incollision;
    }

}
}

#endif /* OBVTREECOLLIDERDFS_HPP_ */
