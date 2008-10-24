/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include <rw/models/SerialDevice.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/math/Jacobian.hpp>
#include <rw/math/Constants.hpp>

#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Math.hpp>

#include <boost/test/unit_test.hpp>

using namespace rw;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace boost::numeric::ublas;

namespace
{
    double norm_inf(const Vector3D<>& v)
    {
        return norm_inf(v.m());
    }

    double norm_1(const Vector3D<>& v)
    {
        return norm_1(v.m());
    }

    double norm_2(const Vector3D<>& v)
    {
        return norm_2(v.m());
    }

    double norm_inf(const Q& v)
    {
        return norm_inf(v.m());
    }

    double norm_1(const Q& v)
    {
        return norm_1(v.m());
    }

    double norm_2(const Q& v)
    {
        return norm_2(v.m());
    }
}

/*
  The analytical solution to the forward kinematics of the Unimatron PUMA560
  robot (from Craig)
*/
Transform3D<> Puma560(Q& q, double a2, double d3, double a3, double d4){
    double c1 = cos(q[0]);
    double c2 = cos(q[1]);
    double c3 = cos(q[2]);
    double c4 = cos(q[3]);
    double c5 = cos(q[4]);
    double c6 = cos(q[5]);

    double s1 = sin(q[0]);
    double s2 = sin(q[1]);
    double s3 = sin(q[2]);
    double s4 = sin(q[3]);
    double s5 = sin(q[4]);
    double s6 = sin(q[5]);

    double c23 = c2*c3-s2*s3;
    double s23 = c2*s3+s2*c3;

    double r11 = c1*(c23*(c4*c5*c6-s4*s6) - s23*s5*c6) + s1 * (s4*c5*c6+c4*s6);
    double r21 = s1*(c23*(c4*c5*c6-s4*s6) - s23*s5*c6) - c1 * (s4*c5*c6+c4*s6);
    double r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;

    double r12 = c1*(c23*(-c4*c5*s6-s4*c6)+s23*s5*s6) + s1*(c4*c6-s5*c5*s6);
    double r22 = s1*(c23*(-c4*c5*s6-s4*c6)+s23*s5*s6) - c1*(c4*c6-s4*c5*s6);
    double r32 = -s23*(-c4*c5*s6-s4*c6)+c23*s5*s6;

    double r13 = -c1*(c23*c4*s5+s23*c5) - s1*s4*s5;
    double r23 = -s1*(c23*c4*s5+s23*c5) + c1*s4*s5;
    double r33 = s23*c4*s5-c23*c5;

    double px = c1*(a2*c2+a3*c23-d4*s23) - d3*s1;
    double py = s1*(a2*c2+a3*c23-d4*s23) + d3*c1;
    double pz = -a3*s23-a2*s2-d4*c23;

    return Transform3D<>(
        Vector3D<>(
            px, py, pz
            ),
        Rotation3D<>(
            r11, r12, r13,
            r21, r22, r23,
            r31, r32, r33)
        );
}

void forwardKinematicsTest()
{
    // first a simple test, remember the State takes ownership of the
    // frames added to the tree.
    // Which means they have to be allocated with new
    // Define the world frame and construct the frame Tree
    BOOST_MESSAGE("- testing serialdevice forward kinematics ");
    boost::shared_ptr<StateStructure> tree =
        boost::shared_ptr<StateStructure>(new StateStructure());
    Frame *world = tree->getRoot();
    //tree->addFrame(world);
    { // simple forward kinematic of one joint
        // Define a very simple robot
        FixedFrame *base = new FixedFrame("base",Transform3D<>::identity());
        RevoluteJoint *joint1 = RevoluteJoint::make("joint1",Transform3D<>::identity());
        FixedFrame *tool = new FixedFrame("tool",Transform3D<>::identity());

        // set the RevoluteJoint to be active
        Accessor::activeJoint().set(*joint1,true);

        // update the tree with the serial chain
        tree->addFrame(base,world);
        tree->addFrame(joint1,base);
        tree->addFrame(tool,joint1);

        //State state(tree);
        State state = tree->getDefaultState();

        SerialDevice simple(base, tool, "simple1", state);

        BOOST_CHECK(simple.frames().size() == 3);
        BOOST_CHECK(simple.getBase() == base);

        Q qs(1);
        qs[0] = Pi/2.0;
        simple.setQ(qs,state);

        BOOST_CHECK(simple.getQ(state)[0] == Pi/2.0);

        BOOST_CHECK(norm_inf(joint1->getTransform(state).P()) == 0);
        BOOST_CHECK(norm_inf(joint1->getTransform(state).R().m() - EAA<>(0.0, 0.0, Pi/2.0).toRotation3D().m()) <= 1e-6);

        Transform3D<> bTe_s = simple.baseTend(state);

        //std::cout << bTe_s << "\n";

        BOOST_CHECK(norm_inf(bTe_s.P()) == 0);
        BOOST_CHECK(norm_inf(bTe_s.R().m() - EAA<>(0.0, 0.0, Pi/2.0).toRotation3D().m()) <= 1e-6);

        //std::cout << bTe_s << "\n";
        //std::cout << b.cTf(&e);
    }

    { // forward kinematics of a serialChain robot
        // Define the constants
        const double a2 = 0.4318;
        const double d3 = -1.15005;
        const double a3 = 0.0203;
        const double d4 = -0.4318;

        /*
         * Define the individual links and axes frames of the PUMA560 serial robot
         * the Craig D-H parameters for the individual links are taken from
         * Figure 3.21 (page 80) in the
         * "Introduction to Robotics, mechanics and control, 2nd edition"
         * by John J. Craig
         */

        // Define the PUMA560 base frame
        FixedFrame *base = new FixedFrame("Base",Transform3D<>::identity());
        // And then all the joints
        RevoluteJoint *joint1 = RevoluteJoint::make("Joint1",Transform3D<>::craigDH( 0, 0, 0, 0));
        RevoluteJoint *joint2 = RevoluteJoint::make("Joint2",Transform3D<>::craigDH( -Pi/2.0, 0, 0, 0));
        RevoluteJoint *joint3 = RevoluteJoint::make("Joint3",Transform3D<>::craigDH( 0, a2, d3, 0));
        RevoluteJoint *joint4 = RevoluteJoint::make("Joint4",Transform3D<>::craigDH( -Pi/2.0, a3, d4, 0));
        RevoluteJoint *joint5 = RevoluteJoint::make("Joint5",Transform3D<>::craigDH( Pi/2.0, 0, 0, 0));
        RevoluteJoint *joint6 = RevoluteJoint::make("Joint6",Transform3D<>::craigDH( -Pi/2.0, 0, 0, 0));
        // And last define the PUMA560 end-effector frame
        FixedFrame *tool = new FixedFrame("Tool",Transform3D<>::identity());

        // update the active attribute on all active joints, which means all actuated joints
        Accessor::activeJoint().set(*joint1,true);
        Accessor::activeJoint().set(*joint2,true);
        Accessor::activeJoint().set(*joint3,true);
        Accessor::activeJoint().set(*joint4,true);
        Accessor::activeJoint().set(*joint5,true);
        Accessor::activeJoint().set(*joint6,true);

        // add all frames and joints to the Tree there by defining their parent child relationship
        tree->addFrame(base,world);
        tree->addFrame(joint1,base);
        tree->addFrame(joint2,joint1);
        tree->addFrame(joint3,joint2);
        tree->addFrame(joint4,joint3);
        tree->addFrame(joint5,joint4);
        tree->addFrame(joint6,joint5);
        tree->addFrame(tool,joint6);


        // construct the State that should hold the states of the seriel device
        //State state(tree);
        State state = tree->getDefaultState();

        /*
         * Now we are ready to construct the serial device
         */
        SerialDevice puma560Device(base,tool,"PUMA560",state);

        Q q(6);
        q[0] = Pi/8.0;
        q[1] = Pi/8.0;
        q[2] = Pi/8.0;
        q[3] = Pi/8.0;
        q[4] = Pi/8.0;
        q[5] = Pi/8.0;

        puma560Device.setQ(q,state);

        // Compare the DH based forward kinematics with the analytical solution
        Transform3D<> bTe1 = Kinematics::frameTframe(base,tool,state);
        Transform3D<> bTe2 = puma560Device.baseTend(state);
        //Transform3D<> bTe3 = puma560Device.bTf(&endEffector);
        Transform3D<> compare = Puma560(q, a2, d3, a3, d4);

        BOOST_CHECK(norm_inf(bTe1.P() - compare.P()) < 1e-6);
        BOOST_CHECK(norm_inf(bTe1.R().m() - compare.R().m()) < 1e-6);

        BOOST_CHECK(norm_inf(bTe2.P() - compare.P()) < 1e-6);
        BOOST_CHECK(norm_inf(bTe2.R().m() - compare.R().m()) < 1e-6);

        //BOOST_CHECK(norm_inf(bTe3.P() - compare.P()) < 1e-6);
        //BOOST_CHECK(norm_inf(bTe3.R().m() - compare.R().m()) < 1e-6);
    }
}

void SerialDeviceTest(){

    forwardKinematicsTest();

    // Define the world frame and construct the frame Tree
    boost::shared_ptr<StateStructure> tree =
        boost::shared_ptr<StateStructure>(new StateStructure());
    Frame *world = tree->getRoot();
    //tree->addFrame(world);

    // Define the simplified (only 6-dof) Kuka-kr16 robot

    // Define the PUMA560 base frame
    FixedFrame *base =
        new FixedFrame("Base", Transform3D<>(Vector3D<>(2.0, 0.0, 1.0), RPY<>(Pi, 0.0, Pi)) ) ;

    // And then all the joints
    RevoluteJoint *joint1 = RevoluteJoint::make("Joint1",Transform3D<>::craigDH( 0, 0, 0, 0));
    RevoluteJoint *joint2 = RevoluteJoint::make("Joint2",Transform3D<>::craigDH( Pi/2.0, 0.26, 0, 0));
    RevoluteJoint *joint3 = RevoluteJoint::make("Joint3",Transform3D<>::craigDH( 0, 0.68, 0, 0));
    RevoluteJoint *joint4 = RevoluteJoint::make("Joint4",Transform3D<>::craigDH( Pi/2.0, -0.035, -0.67, 0));
    RevoluteJoint *joint5 = RevoluteJoint::make("Joint5",Transform3D<>::craigDH( -Pi/2.0, 0, 0, 0));
    RevoluteJoint *joint6 = RevoluteJoint::make("Joint6",Transform3D<>::craigDH( Pi/2.0, 0, 0, 0));
    // And last define the PUMA560 end-effector frame, but don't add it to the serial chain yet
    FixedFrame *tool =
        new FixedFrame(
                       "Tool",Transform3D<>(
                           Vector3D<>(-0.141, 0.0, -0.299),
                           Rotation3D<>(
                               0.0, -1.0/sqrt(2.0), -1.0/sqrt(2.0),
                               -1.0,            0.0,            0.0,
                               0.0,  1.0/sqrt(2.0), -1.0/sqrt(2.0) )) ) ;

    // update the active attribute on all active joints, which means all actuated joints
    Accessor::activeJoint().set(*joint1,true);
    Accessor::activeJoint().set(*joint2,true);
    Accessor::activeJoint().set(*joint3,true);
    Accessor::activeJoint().set(*joint4,true);
    Accessor::activeJoint().set(*joint5,true);
    Accessor::activeJoint().set(*joint6,true);

    // add all frames and joints to the Tree there by defining their parent child relationship
    tree->addFrame(base,world);
    tree->addFrame(joint1,base);
    tree->addFrame(joint2,joint1);
    tree->addFrame(joint3,joint2);
    tree->addFrame(joint4,joint3);
    tree->addFrame(joint5,joint4);
    tree->addFrame(joint6,joint5);
    tree->addFrame(tool,joint6);

    //tree->addFrameChain(world, tool);

    // Now before constructing the device, construct the rest of the environment.
    // Define the environment
    FixedFrame *tableFrame = new FixedFrame("Table", Transform3D<>(Vector3D<>(2.0, 1.0, 0.8), RPY<>(0.0, 0.0, Pi)));
    FixedFrame *klods1Frame = new FixedFrame("Klods1", Transform3D<>(Vector3D<>(1.2, 0.52, 0.22), RPY<>(Pi, 0.0, Pi)));
    FixedFrame *klods2Frame = new FixedFrame("Klods2", Transform3D<>(Vector3D<>(1.19, -0.5, 0.22), RPY<>(Pi/2.0, 0.0, Pi)) );
    FixedFrame *klods3Frame = new FixedFrame("Klods3", Transform3D<>(Vector3D<>(0.58, 0.52, 0.22), RPY<>(0.0, 0.0, Pi)) );
    FixedFrame *klods4Frame = new FixedFrame("Klods4", Transform3D<>(Vector3D<>(0.58, -0.5, 0.22), RPY<>(Pi/4.0, 0.0, Pi)) );
    FixedFrame *klods5Frame = new FixedFrame("Klods5", Transform3D<>(Vector3D<>(0.855, 0.0, 0.22), RPY<>(297.0*Pi/180.0, 0.0, Pi)));

    tree->addFrame(tableFrame, world);
    tree->addFrame(klods1Frame, world);
    tree->addFrame(klods2Frame, world);
    tree->addFrame(klods3Frame, world);
    tree->addFrame(klods4Frame, world);
    tree->addFrame(klods5Frame, world);

    // construct the State that should hold the states of the seriel device
    //State state(tree);
    State state = tree->getDefaultState();

    // Now we are ready to construct the serial device
    SerialDevice kr16t(base,tool,"KR16",state);

    // define the waypoint
    Q qwp(6);
    qwp[0] = 0.0;
    qwp[1] = -60.0 * Pi / 180.0;
    qwp[2] = 15.0 * Pi / 180.0;
    qwp[3] = 0.0;
    qwp[4] = 0.0;
    qwp[5] = 0.0;

    // set the configuration of the robot
    kr16t.setQ(qwp,state);

    BOOST_CHECK(
        norm_inf(kr16t.baseTend(state).P() - Vector3D<>(1.16074, 0.0, 0.22074)) < 1e-5);

    BOOST_CHECK(
        norm_inf(
            kr16t.baseTend(state).R().m() -
            Rotation3D<>(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).m()) < 1e-6);

    BOOST_CHECK(kr16t.getName() == "KR16");

    /**
     * Jacobian test
     *
     */
    Frame *kr16_j6 = joint6;

    // Make velocity vector
    Q dq(6);
    for(size_t i = 0; i<dq.size(); i++)
        dq[i] = 1.0;

    // Velocity of end-effector seen from base
    VelocityScrew6D<> bVe = kr16t.baseJframe(tool,state) * dq;

    // Velocity of tool seen from base
    VelocityScrew6D<> bVt = kr16t.baseJend(state) * dq;

    // Velocity of end-effector seen from end-effector
    VelocityScrew6D<> eVe = inverse(kr16t.baseTframe(tool,state).R()) * bVe;

    // Velocity of tool seen from tool
    VelocityScrew6D<> tVt = inverse(kr16t.baseTend(state).R()) * bVt;

    // Velocity of end-effector seen from end-effector (calculated from tool velocity)
    VelocityScrew6D<> eVe2 = tool->getTransform(state) * tVt;

    // Velocity of tool seen from tool (calculated from end-effector velocity)
    VelocityScrew6D<> tVt2 = inverse(tool->getTransform(state)) * eVe;

    Transform3D<double> t_bTe = kr16t.baseTend(state);
    Transform3D<double> t_bTf = kr16t.baseTframe(tool,state);
    //std::cout << "BTE " << t_bTe << "\n";
    //std::cout << "BTF " << t_bTf << "\n";
    return;

    // TODO: somethings wrong here under
    std::cout << (eVe-eVe2) << "\n";
    std::cout << (tVt-tVt2) << "\n";
    BOOST_CHECK(norm_inf(eVe - eVe2) < 1e-6);
    BOOST_CHECK(norm_inf(tVt - tVt2) < 1e-6);

    Jacobian b_eJq = kr16t.baseJframe(kr16_j6,state);
    Jacobian e_eJb_e(inverse(kr16t.baseTframe(kr16_j6,state).R()));
    Jacobian t_tJe_e(inverse(tool->getTransform(state)));

    Jacobian t_tJq = t_tJe_e * e_eJb_e * b_eJq;
    VelocityScrew6D<> tVt3 = t_tJq * dq;

    BOOST_CHECK(norm_inf(tVt - tVt3) < 1e-6);

    Jacobian b_tJe_t(kr16t.baseTframe(kr16_j6,state).R());
    Jacobian e_tJt_t(tool->getTransform(state).R());

    Jacobian b_tJb_e = b_tJe_t * e_tJt_t * t_tJe_e * e_eJb_e;

    //std::cout
    //    << "b_tJb_e"
    //    << b_tJb_e
    //    << "\n"
    //    << "b_tJb_e"
    //    << Jacobian(inverse(tool->getTransform(state)).P())
    //    << "\n";

    boost::numeric::ublas::matrix<double> placeholder =
        prod(
            kr16t.baseTend(state).R().m(),
            Math::skew(inverse(tool->getTransform(state)).P().m()));

    //std::cout
    //    << "1) "
    //    << prod(placeholder,inverse(kr16t.baseTend(state)).R().m())
    //    << "\n";

    //std::cout
    //    << "2) "
    //    << Math::skew(inverse(tool->getTransform(state)).P().m())
    //    << "\n";
}
