#include "HGraspPlanner3D.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/ResolvedRateSolver.hpp>
#include <rw/invkin/SimpleMultiSolver.hpp>

#include <boost/foreach.hpp>

#include <invkin/CG3IKSolver2D.hpp>
#include <geometry/Contour2DUtil.hpp>

#include "SimpleMeasure.hpp"

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;


HGraspPlanner3D::HGraspPlanner3D(
        const rw::models::TreeDevice& hand,
        rw::models::JointDevice& robot,
        rw::proximity::CollisionDetector& colDect,
        const rw::kinematics::State& state):
            _hand(hand), _robot(robot),
            _graspGen(hand, state),
            _detector(&colDect),
            _hbTwrist(Kinematics::frameTframe(hand.getBase(),robot.getEnd(),state)),
            _clerance(0.03),
            _maxObjHeight(0.07),
            _minQuality(0.5),
            _graspHeight(0.04),
            _maxCurvature(0.05),
            _maxAppAngle(8*Deg2Rad)

{

}

HGraspPlanner3D::HGraspPlanner3D(
        const rw::models::TreeDevice& hand,
        rw::models::JointDevice& robot,
        const rw::kinematics::State& state):
            _hand(hand), _robot(robot),
            _graspGen(hand, state),
            _detector(NULL),
            _hbTwrist(Kinematics::frameTframe(hand.getBase(),robot.getEnd(),state)),
            _clerance(0.03),
            _maxObjHeight(0.07),
            _minQuality(0.5),
            _graspHeight(0.04),
            _maxCurvature(0.05),
            _maxAppAngle(8*Deg2Rad)
{

}

std::vector<GraspResult> query(const rw::kinematics::State& state, int maxNrOfQs=1){

    // 1. contact generator
    // 2. apply fast filter rules
    // 3. apply quality filter rules
    // 4. apply collision free inverse kinematics calculation
    // 5. return the N best grasps
    std::vector<GraspResult> result;

    DiceContactG3D _generator;

    do {
        DContact3D con = _generator.generateNext();
        // the contact is allready half tested for force closure.. so apply some other filter.
        bool goodContact = false;
        BOOST_FOREACH(ContactFilter *filter, _cfilters){
            goodContact &= filter->check(con);
            if( !goodContact )
                break;
        }
        if(!goodContact)
            continue;
    } while( result.size()<maxNrOfQs );

    return result;
}
