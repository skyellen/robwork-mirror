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

#ifndef RW_PROXIMITY_BVTREECOLLIDERFACTORY_HPP_
#define RW_PROXIMITY_BVTREECOLLIDERFACTORY_HPP_

#include <rw/geometry/OBB.hpp>
#include <rw/geometry/OBBCollider.hpp>
#include <rw/geometry/TriTriIntersectDeviller.hpp>

//#include "OBBToleranceCollider.hpp"
#include "BVTreeCollider.hpp"
#include "OBVTreeDFSCollider.hpp"


namespace rw {
namespace proximity {

    /**
     * @brief factory for creating tree colliders.
     */
    class BVTreeColliderFactory {
    public:

        /**
         * @brief template base class to deside which node to descent into.
         */
        template<class DERIVED>
        struct BVDescentStrategy {
            typedef typename Traits<DERIVED>::BVType BVType;
            typedef typename Traits<DERIVED>::StateType StateType;

            /**
             * @brief returns true if
             * @param bvA
             * @param bvB
             * @param state
             * @return
             */
            inline bool descentIntoA(const BVType& bvA, const BVType& bvB, StateType& state){
                return static_cast<DERIVED*>(this)->descentIntoA(bvA,bvB, state);
            }

        };


        /**
         * @brief balanced descent strategy. The previous descent choice is saved and the oposite is choosen if
         */
        template<class BVTREE>
        struct BalancedDescentStrategy: public BVDescentStrategy<BalancedDescentStrategy<BVTREE> >{
            typedef typename Traits<BVTREE>::BVType BVType;
            //! returns the oposite direction than the previous state.
            inline bool descentIntoA(const BVType& bvA, const BVType& bvB, bool& state){
                state = !state;
                return state;
            }
        };

        template<class BVTREE>
        struct MaxAreaDescentStrategy: public BVDescentStrategy<MaxAreaDescentStrategy<BVTREE> >{
            typedef typename Traits<BVTREE>::BVType BVType;
            //! returns the oposite direction than the previous state.
            inline bool descentIntoA(const BVType& bvA, const BVType& bvB, bool& state){
                return bvA.calcArea() > bvB.calcArea();
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
            rw::geometry::OBBCollider<typename BVTREE::value_type>* bvcollider = new rw::geometry::OBBCollider<typename BVTREE::value_type>();
            BalancedDescentStrategy<BVTREE>* dstrategy = new BalancedDescentStrategy<BVTREE>();
            return makeDFSCollider<BVTREE>(bvcollider, dstrategy);
        }

        /**
         * @brief creates a depth first search tree collider for trees that use triangles for
         * primitives.
         * @param bvcollider
         * @param dstrat
         * @return
         */
        template<class BVTREE, class COLLIDER, class DESCENT_STRATEGY>
        static BVTreeCollider<BVTREE>* makeDFSCollider(COLLIDER* bvcollider, DESCENT_STRATEGY* dstrat){
            typedef typename BVTREE::value_type T;
            rw::geometry::TriTriIntersectDeviller<T> *primCollider = new rw::geometry::TriTriIntersectDeviller<T>();
            return makeDFSCollider<BVTREE>(bvcollider, primCollider, dstrat);
        }

        /**
         * @brief creates a depth first search tree collider for trees that use triangles for
         * primitives.
         * @param bvcollider
         * @param dstrat
         * @return
         */
        template<class BVTREE, class COLLIDER, class DESCENT_STRATEGY, class PRIMCOLLIDER>
        static BVTreeCollider<BVTREE>* makeDFSCollider(COLLIDER* bvcollider, PRIMCOLLIDER* primcollider, DESCENT_STRATEGY* dstrat){
            return new OBVTreeDFSCollider<BVTREE, COLLIDER, DESCENT_STRATEGY, PRIMCOLLIDER>(bvcollider, primcollider, dstrat);
        }

        /**
         * @brief creates a tree collider that performes depth first search
         * of bv collisions between two hierarchical trees. The search is
         * balanced in the way that it equally switches between descending in the
         * trees.
         */
        /*
        template<class BVTREE>
        static std::pair<BVTreeCollider<BVTREE>*,OBBToleranceCollider<typename BVTREE::value_type>* > makeBalancedDFSToleranceColliderOBB(){
            typedef typename BVTREE::value_type T;
            OBBToleranceCollider<T>* bvcollider = new OBBToleranceCollider<T>();
            BalancedDescentStrategy<BVTREE>* dstrategy = new BalancedDescentStrategy<BVTREE>();
            return make_pair( makeDFSCollider<OBBToleranceCollider<T>, BalancedDescentStrategy<BVTREE>, BVTREE>(bvcollider, dstrategy), bvcollider);
        }
        */


        //static BVTreeCollider<BinaryOBBPtrTreeD>* makeOBBPtrTreeBDFSColliderD();

        //static BVTreeCollider<BinaryOBBPtrTreeF>* makeOBBPtrTreeBDFSColliderF();

    private:

    };

}

template<class BVTREE>
struct Traits<proximity::BVTreeColliderFactory::BalancedDescentStrategy<BVTREE> > {
    typedef BVTREE BVTree;
    typedef typename Traits<BVTREE>::BVType BVType;
    typedef bool StateType;
};

template<class BVTREE>
struct Traits<proximity::BVTreeColliderFactory::MaxAreaDescentStrategy<BVTREE> > {
    typedef BVTREE BVTree;
    typedef typename Traits<BVTREE>::BVType BVType;
    typedef bool StateType;
};

}

#endif /* BVTREECOLLIDERFACTORY_HPP_ */
