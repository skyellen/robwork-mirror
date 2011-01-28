#ifndef RWSIM_SIMULATOR_CONTACTMODELFACTORY_HPP_
#define RWSIM_SIMULATOR_CONTACTMODELFACTORY_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

#include <rw/models/WorkCell.hpp>

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/DynamicWorkcell.hpp>

#include "ContactModel.hpp"
#include "ConstraintEdge.hpp"
#include "ConstraintNode.hpp"
#include "CNodePool.hpp"


namespace rwsim {
namespace simulator {


    struct FilteredPoint;

    /**
     * @brief The ContactModelFactory functions as a mediator to the functionallity
     * that is required by the contact graph.
     *  - Broad and Narrow phase collision detection
     *  - Contact and collision determination
     *  - Creating constraints between constraint nodes
     */
    class ConstraintFactory
    {
    public:

        /**
         * @brief Default destructor
         */
        virtual ~ConstraintFactory(){};

        /**
         * @brief calculates the broad phase contact information and adds all contacting
         * frames to the frame pair list oFrames
         */
        virtual void broadPhaseCalc(rw::kinematics::State &state,
                                    rw::kinematics::FramePairSet &oFrames);

        /**
         * @brief calculates narrow phase contact information, and adds it to the edge
         */
        virtual void narrowPhaseCalc(ConstraintEdge& edge,
                                     rw::kinematics::State &state);

        /**
         * @brief Determines the contact information proximity info
         * located in the Edge.
         */
        virtual void DetermineContact(ConstraintEdge &edge, rw::kinematics::State& state);

        /**
         * @brief creates a ConstraintEdge from a pair of nodes
         */
        virtual ConstraintEdge* CreateConstraintEdge(CNodePair& nodePair);

        /**
         * @brief constructs an edge using the above method, but sorts the nodes
         * such that the first node in the pair has the lowest pointer value.
         */
        ConstraintEdge* CreateConstraintEdge(ConstraintNode* nodeA,ConstraintNode* nodeB){
            CNodePair pair(nodeA,nodeB);
            return CreateConstraintEdge(pair);
        }

        /**
         * @brief The the maximal allowed distance between two touching contacts
         */
        double getTouchDist(){ return _touchDist; }

        /**
         * @brief The minimal allowed distance between two contacts
         */
        double getPenetrationDist(){ return _penDist; }


    private:
        ContactModelFactory();

        dynamics::DynamicWorkCell *_dwc;

        CNodePool *_pool;

        std::list<FilteredPoint> _filteredPoints;

        rwlibs::proximitystrategies::ProximityStrategyPQP *_narrowStrategy;

        rw::proximity::CollisionDetector *_toleranceDetector;

        double _touchDist, _penDist, _sepDist;
    };

    struct FilteredPoint {
    public:
        FilteredPoint(const rw::math::Vector3D<>& a,
                const rw::math::Vector3D<>& b,
                double d):
                    p(a),p1(a),p2(b),dist(d),hits(0)
        {}

        rw::math::Vector3D<> p;
        rw::math::Vector3D<> p1;
        rw::math::Vector3D<> p2;
        double dist;
        int hits;
    };
}
}
#endif /*CONTACTMODELFACTORY_HPP_*/
