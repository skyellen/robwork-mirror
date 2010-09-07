#include "SDHGraspPlanner2D.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/invkin/IKMetaSolver.hpp>
#include <rw/invkin/ResolvedRateSolver.hpp>
#include <rw/invkin/SimpleMultiSolver.hpp>

#include <boost/foreach.hpp>

#include "CG3IKSolver2D.hpp"
#include <rw/geometry/Contour2DUtil.hpp>

#include "SimpleMeasure.hpp"

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::invkin;
using namespace rw::graspplanning;
using namespace rw::geometry;


#define MAX_BAD_GRASPS 5

SDHGraspPlanner2D::SDHGraspPlanner2D(
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

SDHGraspPlanner2D::SDHGraspPlanner2D(
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


void SDHGraspPlanner2D::setContour(
    const Contour2D& contour,
    const rw::math::Transform3D<>& baseTcontour)
{
    // Initialize grasp candidate generator
    _baseTcontour = baseTcontour;
    //_graspGen.setMaxCurvature(_maxCurvature);
    //_graspGen.setPerpFilter(_maxAppAngle);
    _graspGen.init(contour, 100, 30);
}


std::vector<SDHGraspPlanner2D::GraspResult>
    SDHGraspPlanner2D::query(const State& initState, int maxNrOfQs){

    std::cout << "SDHGraspPlanner2D: query - " << std::endl
              << " - approach clerance: " << _clerance << "m" << std::endl
              << " - grasp height: " << _graspHeight << "m" << std::endl
              << " - max obj height: " << _maxObjHeight << "m" <<  std::endl
              << " - max curvature: " << _maxCurvature*Rad2Deg << "Deg" << std::endl
              << " - max approach/normal angle: " << _maxAppAngle*Rad2Deg << "Deg" << std::endl
              << " - min quality: " << _minQuality << std::endl;

    State state = initState;
    std::vector<SDHGraspPlanner2D::GraspResult> graspResults;
    // create the quality measures of choice
    SimpleMeasure measure(0.3, _graspGen.getContour().center() );
    // get
    _grasps = _graspGen.computeGrasps(measure, _minQuality, maxNrOfQs);
    if(_grasps.size()==0) {
        std::cout << "SDHGraspPlanner2D: No grasp candidates found!" << std::endl;
        return graspResults;
    }

    // get the transform from robot base to table reference frame
    const Transform3D<> &bTtable = _baseTcontour;
    // and make sure that we grasp it above the table
    Transform3D<> tableTgrasp(Vector3D<>(0,0,_graspHeight));
    const Transform3D<> bTgrasp = bTtable * tableTgrasp;

    // create the inverse kinematics solvers
    CG3IKSolver2D ikHandSolver(_hand, state);
    //ikHandSolver.setMinWristHeight( _maxObjHeight - 2 );

    ResolvedRateSolver ikRobotSolver(&_robot, state);

    IKMetaSolver ikMetaSolver(&ikRobotSolver, &_robot, _detector);

    // Stuff for printing nice stats
    int nrCandGrasp = _grasps.size();
    int nrAppIk(0), nrAppIkRobot(0);
    int nrIk(0), nrIkRobot(0),nrBadGrasps(0);

    BOOST_FOREACH(const Grasp2D& grasp, _grasps){
        // first create the approach grasp and its inverse kinematics solution
        // make sure that the approach grasp is collision free, so test a few clerances before discarding an approach grasp
        CG3IKSolver2D::IKResult ikHandRes;
        Grasp2D approachGrasp = grasp;
        std::vector<Q> ikMetaRes;
        int badGrasps = 0;
        for(badGrasps=0;badGrasps<MAX_BAD_GRASPS;badGrasps++){
            int i=badGrasps;
            approachGrasp = grasp;
            approachGrasp.scale(_clerance+i*(0.02/MAX_BAD_GRASPS));

            ikHandRes.clear();
            ikHandRes = ikHandSolver.solve(bTgrasp, approachGrasp, state);
            if( ikHandRes.size()==0 )
                continue;
            nrAppIk++;
            _hand.setQ(ikHandRes[0].second,state);
            // then try if the robot can reach the wrist configuration without collision
            // and without breaking joint limits

            // TODO: possibly test multiple hand configurations
            ikMetaRes.clear();
            //ikMetaRes= ikRobotSolver.solve(ikHandRes[0].first*_hbTwrist, state);
            ikMetaRes = ikMetaSolver.solve( ikHandRes[0].first*_hbTwrist , state, 20, true );
            //ikMetaSolver.solve( ikHandRes[0].first*_hbTwrist , state, 200, true );
            if( ikMetaRes.size()==0 )
                continue;
            nrAppIkRobot++;
            break;
        }
        nrBadGrasps += badGrasps;
        if(badGrasps==MAX_BAD_GRASPS)
            continue;

        Q qHandApp = ikHandRes[0].second;
        Q qRobotApp = ikMetaRes[0];

        // now find the ik for the contact grasp
        ikHandRes = ikHandSolver.solve(bTgrasp, grasp, state);
        if( ikHandRes.size()==0 )
            continue;
        nrIk++;

        // find an ik for the contact grasp that is close to the approach configuration
        _robot.setQ(qRobotApp, state);
        std::vector<Q> ikRobotRes = ikRobotSolver.solve(ikHandRes[0].first*_hbTwrist, state);
        if( ikRobotRes.size()==0 )
            continue;
        nrIkRobot++;

        Q qHand = ikHandRes[0].second;
        Q qRobot = ikRobotRes[0];

        Q approachQ(qHand.size()+qRobot.size());
        Q graspQ(qHand.size()+qRobot.size());

        for(size_t i=0;i<qRobot.size(); i++){
            approachQ[i] = qRobotApp[i];
            graspQ[i] = qRobot[i];
        }
        const int n = qRobot.size();
        for(size_t i=0;i<qHand.size(); i++){
            approachQ[i+n] = qHandApp[i];
            graspQ[i+n] = qHand[i];
        }

        graspResults.push_back(std::make_pair(approachQ,graspQ));
    }

    std::cout << "SDHGraspPlanner2D: result " << std::endl
              << " - nr of candidate grasps: " << nrCandGrasp << std::endl
              << " - grasps after approach ik filter: " << nrAppIk << std::endl
              << " - grasps after meta ik filter: " << nrAppIkRobot << std::endl
              << " - nr of bad approach grasps: " << nrBadGrasps << std::endl
              << " - grasps after grasp contact ik filter: " << nrIk << std::endl
              << " - grasps after ik contact filter: " << nrIkRobot << std::endl;
    return graspResults;
}

