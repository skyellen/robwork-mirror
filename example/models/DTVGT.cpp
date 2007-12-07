/*
 * An example of modelling a parallel DTVGT robot and calculating forward kinematics
 */

#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>

#include <rw/kinematics/Tree.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/util.hpp>
#include <rw/models/ParallelLeg.hpp>
#include <rw/models/ParallelDevice.hpp>


using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;

std::vector<Frame*> legVGT(Frame* parent,Transform3D<> base, Transform3D<> tool, boost::shared_ptr<Tree> &tree){
    std::vector<Frame*> leg;
    /*
    q[0] = 0.0 ;//M_PI/4.0;
    q[1] = 0.0;
    q[2] = 0.0; //-M_PI/2.0;
    q[3] = 0.0;
    q[4] = 0.0; // M_PI/4.0;
    */
    leg.push_back(new FixedFrame(parent,"Base",base));
/*    leg.push_back(new RevoluteJoint("joint1",Transform3D<>::CraigDH( 0, 0, 0, M_PI/4.0 )) );
    leg.push_back(new RevoluteJoint("joint2",Transform3D<>::CraigDH( M_PI/2.0, 50, 0, 0 )) );
    leg.push_back(new RevoluteJoint("joint3",Transform3D<>::CraigDH( -M_PI/2.0, 50, 0, -M_PI/2.0 )) );
    leg.push_back(new RevoluteJoint("joint4",Transform3D<>::CraigDH( M_PI/2.0, 50, 0, 0 )) );
    leg.push_back(new RevoluteJoint("joint5",Transform3D<>::CraigDH( -M_PI/2.0, 50, 0, M_PI/4.0 )) );
*/
    leg.push_back(new RevoluteJoint(leg.back(),"joint1",Transform3D<>::CraigDH(  M_PI/2.0,   0.0, 0.0, M_PI/4.0 )) );
    leg.push_back(new RevoluteJoint(leg.back(),"joint2",Transform3D<>::CraigDH( -M_PI/2.0, 100.0, 0.0, 0.0 )) );
    leg.push_back(new RevoluteJoint(leg.back(),"joint3",Transform3D<>::CraigDH(  M_PI/2.0,   0.0, 0.0, -M_PI/2.0 )) );
    leg.push_back(new RevoluteJoint(leg.back(),"joint4",Transform3D<>::CraigDH( -M_PI/2.0, 100.0, 0.0, 0.0 )) );
    leg.push_back(new RevoluteJoint(leg.back(),"joint5",Transform3D<>::CraigDH(  M_PI/2.0,   0.0, 0.0, M_PI/4.0 )) );
    leg.push_back(new FixedFrame(leg.back(),"extra",    Transform3D<>::CraigDH( -M_PI/2.0,   0.0, 0.0, 0.0 )) );
    leg.push_back(new FixedFrame(leg.back(),"Tool",tool));

    activeJointAccessor().set(*leg[1],true);

    // add all frames and joints to the Tree there by defining their parent child relationship
    tree->addFrameChain(*leg.front(),*leg.back());
//    tree->addFrame(*leg[0]);
//    tree->setParent(*serialChain[0],*world); parent must be set by caller
/*    for(size_t i=1;i<leg.size();i++){
        tree->addFrame(*leg[i]);
        tree->setParent(*leg[i],*leg[i-1]);
    }
*/
    return leg;
}


int main(){

    boost::shared_ptr<Tree> tree = boost::shared_ptr<Tree>(new Tree());
    FixedFrame *world = new FixedFrame(NULL,"World", Transform3D<>::Identity());
    tree->addFrame(*world);

    /*Leg A*/
    Transform3D<> baseTransA1(Vector3D<>(0,-100,0),RPY<>(0,0,0)); // Base A
    //Transform3D<> rotBaseA(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,0.0,M_PI));
    //Transform3D<> baseTransA1 = baseTransAA1*rotBaseA;

    Transform3D<> toolTransA1(Vector3D<>(0,100,0),RPY<>(0,0,0)); // Tool A
    //Transform3D<> rotToolA(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,0.0,-M_PI));
    //Transform3D<> toolTransA1 = rotToolA*toolTransAA1;

    ParallelLeg legA( legVGT(world,baseTransA1,toolTransA1,tree));
//    tree->setParent(*legA.getBase(),*world);

    /*Leg B*/
    Transform3D<> baseTransB1(Vector3D<>(-86.602,50,0),RPY<>(-M_PI/3.0,0,0));
    //Transform3D<> rotBaseB(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,M_PI/2.0,));
    //Transform3D<> baseTransB1 = baseTransBB1*rotBaseB;

    Transform3D<> toolTransB1(Vector3D<>(86.602,-50,0),RPY<>(M_PI/3.0,0,0));
    //Transform3D<> rotToolB(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,-M_PI/2.0,M_PI/3.0));
    //Transform3D<> toolTransB1 = rotToolB*toolTransBB1;

    ParallelLeg legB(legVGT(world,baseTransB1,toolTransB1, tree));
//    tree->setParent(*legB.getBase(),*world);

    /*Leg C*/
    Transform3D<> baseTransC1(Vector3D<>(86.602,50,0),RPY<>(M_PI/3.0,0,0));
    //Transform3D<> rotBaseC(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,M_PI/2.0,M_PI/3.0));
    //Transform3D<> baseTransC1 = baseTransCC1*rotBaseC;

    Transform3D<> toolTransC1(Vector3D<>(-86.602,-50,0),RPY<>(-M_PI/3.0,0,0));
    //Transform3D<> rotToolC(Vector3D<>(0,0,0),Util::ZYXToRotation3D(0.0,-M_PI/2.0,-M_PI/3.0));
    //Transform3D<> toolTransC1 = rotToolC*toolTransCC1;

    ParallelLeg legC(legVGT(world,baseTransC1,toolTransC1, tree));
//    tree->setParent(*legC.getBase(),*world);

    // construct the WorkCellState that should hold the states of the seriel device
    State state(tree);

    // LegA configuration
    Q qV(5);
    qV[0] = 0.0;
    qV[1] = 0.0;
    qV[2] = 0.0;
    qV[3] = 0.0;
    qV[4] = 0.0;
    legA.setQ(qV,state);
    legB.setQ(qV,state);
    legC.setQ(qV,state);

    std::vector<ParallelLeg*> legs;
    legs.push_back(&legA);
    legs.push_back(&legB);
    legs.push_back(&legC);

    // print bTe for all legs, they must be equal
    std::cout << "LegA bTe: " << legA.baseTend(state) << std::endl;
    std::cout << "LegB bTe: " << legB.baseTend(state) << std::endl;
    std::cout << "LegC bTe: " << legC.baseTend(state) << std::endl;

    /*
     * Now we are ready to construct the serial device
     */
//    ParallelDevice triVGT(legs,"TriVGT",state);

/*    // Print end-effector position wrt. base frame
    std::cout << puma560Device.bTe(state) << std::endl;

    // Change the position of 1 joint
    Q q(6);
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    q[4] = 0.0;
    q[5] = M_PI;

    puma560Device.setQinState(q,state);

    // Print the new end-effector position wrt. base frame
    std::cout << puma560Device.bTe(state) << std::endl;
*/
    return 0;
}
