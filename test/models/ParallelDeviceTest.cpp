#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Math.hpp>

#include <rw/models/ParallelDevice.hpp>
#include <rw/models/ParallelLeg.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/iksolvers/ParallelIKSolver.hpp>

#include <rw/kinematics/Tree.hpp>

#include <vector>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;

std::vector<Frame*> ParallelLegHEXAPOD(FixedFrame *base, boost::shared_ptr<Tree> &tree, Transform3D<double> &legBase, Transform3D<double> &tool, double yaw, double pitch){

    std::vector<Frame*> serialChain;
    serialChain.push_back(base);
    serialChain.push_back(new FixedFrame(serialChain.back(),"legOffset",legBase));
    serialChain.push_back(new RevoluteJoint(serialChain.back(),"joint1",Transform3D<>::CraigDH( 0, 0, 0, yaw)));
    serialChain.push_back(new RevoluteJoint(serialChain.back(),"joint2",Transform3D<>::CraigDH( M_PI/2.0, 0, 0, M_PI/2.0+pitch)));
    serialChain.push_back(new PrismaticJoint(serialChain.back(),"joint3",Transform3D<>::CraigDH( M_PI/2.0, 0, 0, 0)));
    serialChain.push_back(new RevoluteJoint(serialChain.back(),"joint4",Transform3D<>::CraigDH( -M_PI/2.0, 0, 0, M_PI/2.0)));
    serialChain.push_back(new RevoluteJoint(serialChain.back(),"joint5",Transform3D<>::CraigDH( M_PI/2.0, 0, 0, -M_PI/2.0)));
    serialChain.push_back(new RevoluteJoint(serialChain.back(),"joint6",Transform3D<>::CraigDH( M_PI/2.0, 0, 0, M_PI/2.0)));
    serialChain.push_back(new FixedFrame(serialChain.back(),"Tool",tool));
    serialChain.push_back(new FixedFrame(serialChain.back(),"End",Transform3D<>::Identity()));

    //activeJointAccessor().set( *(serialChain[2]) , true );
    //activeJointAccessor().set( *(serialChain[3]) , true );
    Accessor::ActiveJoint().set( *(serialChain[4]) , true );
    //activeJointAccessor().set( *(serialChain[5]) , true );
    //activeJointAccessor().set( *(serialChain[6]) , true );
    //activeJointAccessor().set( *(serialChain[7]) , true );

    tree->addFrameChain(serialChain[1], serialChain.back());
    return serialChain;
}

void ParallelDeviceTest()
{
    std::cout << "ParallelDevice Test START" << std::endl;
    { // test of a HexaPod from microbotics

        // initial configuration of legs
        Q qh(6); qh[0] = 0.0; qh[1] = 0.0; qh[2] = 92.6865; qh[3] = 0.0; qh[4] = 0.0; qh[5] = 0.0;

        //SerialDevice kr16(&kr16_base, &kr16_tool, joints, "Kuka", state);
        boost::shared_ptr<Tree> tree = boost::shared_ptr<Tree>(new Tree());
        FixedFrame *world = new FixedFrame(NULL, "World", Transform3D<>::Identity());
        tree->addFrame(world);

        // parameters for the
        double gf[3][2];
        gf[0][0] = -15.371950917173786;
        gf[0][1] = 8.875;
        gf[1][0] = 0.0;
        gf[1][1] = -17.75;
        gf[2][0] = -15.371950917173786;
        gf[2][1] = 8.875;

        Transform3D<> baseTA(Vector3D<>(-60.206416539249155, -11.580306882912305,0),Rotation3D<>(0,0,1,0,-1,0,1,0,0));
        Transform3D<> toolTA(Vector3D<>(14.533675867691082, -8.586935664729255, 5.486283046170201),
                            Math::ZYXToRotation3D<double>(0,-0.5058297976189102, 0.2554802362083838));
        std::vector<Frame*> legA = ParallelLegHEXAPOD(world,tree,baseTA,toolTA,-0.2554802362083838, 0.5058297976189102);

        /*Leg B*/
        Transform3D<> baseTB(Vector3D<>(-40.132048213846446, -46.350132752361176,0),Rotation3D<>(0,0,1,0,-1,0,1,0,0));
        Transform3D<> toolTB(Vector3D<>(-2.6407792002537804, 16.673016630261774, 5.486283046154413),
                             Math::ZYXToRotation3D<double>(0,-0.4486102166464917, 0.3501394675976276));
        std::vector<Frame*> legB = ParallelLegHEXAPOD(world,tree,baseTB,toolTB,-0.3501394675976276,0.4486102166464917);

        /*Leg C*/
        Transform3D<> baseTC(Vector3D<>(40.13204821384643, -46.35013275236119,0),Rotation3D<>(0,0,1,0,-1,0,1,0,0));
        Transform3D<> toolTC(Vector3D<>(2.6407792002537804, 16.673016630261774, 5.486283046154413),
                             Math::ZYXToRotation3D<double>(0,0.4486102166464917, 0.3501394675976276));
        std::vector<Frame*> legC = ParallelLegHEXAPOD(world,tree,baseTC,toolTC,-0.3501394675976276,-0.4486102166464917);

        /*Leg D*/
        Transform3D<> baseTD(Vector3D<>(60.20641653924915, -11.580306882912327, 0),Rotation3D<>(0,0,1,0,-1,0,1,0,0));
         Transform3D<> toolTD(Vector3D<> (-14.533675867691082, -8.586935664729255, 5.486283046170201),
                             Math::ZYXToRotation3D<double>(0,0.5058297976189102, 0.2554802362083838));
        std::vector<Frame*> legD = ParallelLegHEXAPOD(world,tree,baseTD,toolTD,-0.2554802362083838, -0.5058297976189102);

        /*Leg E*/
        Transform3D<> baseTE(Vector3D<> (20.074368325402705, 57.930439635273515, 0),Rotation3D<> (0,0,1,0,-1,0,1,0,0));
        Transform3D<> toolTE(Vector3D<> (-15.112667091527895, -7.521335766866318, 5.486283046171359),
                             Math::ZYXToRotation3D<double>(0,0.05084170375926413, -0.5595869360340129));
        std::vector<Frame*> legE = ParallelLegHEXAPOD(world,tree,baseTE,toolTE,0.5595869360340129, -0.05084170375926413);

        /*Leg F*/
        Transform3D<> baseTF(Vector3D<> (-20.0743683254027, 57.93043963527352, 0),Rotation3D<> (0,0,1,0,-1,0,1,0,0));
        Transform3D<> toolTF(Vector3D<> (15.112667091527895, -7.521335766866318, 5.486283046171359),
                             Math::ZYXToRotation3D<double>(0,-0.05084170375926413, -0.5595869360340129));
        std::vector<Frame*> legF = ParallelLegHEXAPOD(world,tree,baseTF,toolTF,0.5595869360340129, 0.05084170375926413);

//        std::cout << "Construct State" << std::endl;


        State state(tree);

//        std::cout << "State constructed" << std::endl;

        ParallelLeg *legADev = new ParallelLeg(legA);
        legADev->setQ(qh,state);
//        std::cout << "legA" << std::endl;
        ParallelLeg *legBDev = new ParallelLeg(legB);
        legBDev->setQ(qh,state);
//        std::cout << "legB" << std::endl;
        ParallelLeg *legCDev = new ParallelLeg(legC);
        legCDev->setQ(qh,state);
//        std::cout << "legC" << std::endl;
        ParallelLeg *legDDev = new ParallelLeg(legD);
        legDDev->setQ(qh,state);
//        std::cout << "legD" << std::endl;
        ParallelLeg *legEDev = new ParallelLeg(legE);
        legEDev->setQ(qh,state);
//        std::cout << "legE" << std::endl;
        ParallelLeg *legFDev = new ParallelLeg(legF);
        legFDev->setQ(qh,state);
//        std::cout << "legF" << std::endl;

//        std::cout << "Construct ParralelDevice from parallel legs!!" << std::endl;

        // Construct the ParallelDevice from a vector of serial devices (legs)
        std::vector<ParallelLeg*> legs;
        legs.push_back(legADev);
        legs.push_back(legBDev);
        legs.push_back(legCDev);
        legs.push_back(legDDev);
        legs.push_back(legEDev);
        legs.push_back(legFDev);

//        for(size_t i=0; i<legs.size(); i++){
//            std::cout << "leg "<<i<< std::endl << (legs[i]->bTe(state)) << std::endl;
//        }

        ParallelDevice hexapod(legs,"Hexapod",state);

//        std::cout << "Parallel Constructed" << std::endl;

        Q q = hexapod.getQ(state);
//        std::cout << "configuration: " << q << std::endl;
//        std::cout << "Pose 1: "<< hexapod.baseTend(state) << std::endl;

        q[0] += 1;
//        std::cout << "new configuration: " << std::endl;
        hexapod.setQ(q,state);
//		std::cout << "Pose 2: "<< hexapod.getForwardKinematics()->bTe(state) << std::endl;

        q[1] += 2;
        q[2] += 1;
        q[3] += 3;

//        std::cout << "new configuration: " << std::endl;
        hexapod.setQ(q,state);
//		std::cout << "Pose 3: "<< hexapod.getForwardKinematics()->bTe(state) << std::endl;
        Transform3D<> pose = hexapod.baseTend(state);
        Q inv_q = q;

        q[1] += 2;
        q[2] += 1;
        q[3] += 3;
        q[4] += 20;
        q[5] += 3;
//        std::cout << "new configuration: " << std::endl;
        hexapod.setQ(q,state);
//		std::cout << "Pose 4: "<< hexapod.getForwardKinematics()->bTe(state) << std::endl;

//        std::cout << "configuration set... " << std::endl;

//        std::cout << "Test the invers kinematics" << std::endl;
        rw::iksolvers::ParallelIKSolver qsolver(&hexapod);

        std::vector<Q> sol = qsolver.solve(pose,state);
        if(sol.size()!=0){
//            std::cout << "Found   q: " << sol[0] << std::endl;
//            std::cout << "Target  q: " << inv_q << std::endl;
//            std::cout << "Target  p: " << pose << std::endl;

//            std::cout << "Test of inverse kinematics end" << std::endl;
        }
    }

    { // test af d-vgt


    }
    std::cout << "ParallelDevice Test END" << std::endl;
}
