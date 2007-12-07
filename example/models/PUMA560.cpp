/*
 * An example of modelling a PUMA560 serial robot and calculating forward kinematics
 */

#include <rw/math/Transform3D.hpp>

#include <rw/kinematics/Tree.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FixedFrame.hpp>

#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/util.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;

int main(){

    FixedFrame *world = new FixedFrame(NULL,"World", Transform3D<>::Identity());


    // Define the constants for the PUMA560 DH params
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
    FixedFrame *base = new FixedFrame(world,"Base",Transform3D<>::Identity());
    // And then all the joints
    RevoluteJoint *joint1 = new RevoluteJoint(base,"Joint1",Transform3D<>::CraigDH( 0, 0, 0, 0));
    RevoluteJoint *joint2 = new RevoluteJoint(joint1,"Joint2",Transform3D<>::CraigDH( -M_PI/2.0, 0, 0, 0));
    RevoluteJoint *joint3 = new RevoluteJoint(joint2,"Joint3",Transform3D<>::CraigDH( 0, a2, d3, 0));
    RevoluteJoint *joint4 = new RevoluteJoint(joint3,"Joint4",Transform3D<>::CraigDH( -M_PI/2.0, a3, d4, 0));
    RevoluteJoint *joint5 = new RevoluteJoint(joint4,"Joint5",Transform3D<>::CraigDH( M_PI/2.0, 0, 0, 0));
    RevoluteJoint *joint6 = new RevoluteJoint(joint5,"Joint6",Transform3D<>::CraigDH( -M_PI/2.0, 0, 0, 0));
    // And last define the PUMA560 end-effector frame
    FixedFrame *tool = new FixedFrame(joint6,"Tool",Transform3D<>::Identity());

    // add all frames and joints to the Tree there by defining their parent child relationship
    boost::shared_ptr<Tree> tree = boost::shared_ptr<Tree>(new Tree());
    tree->addFrame(*world);
    tree->addFrameChain(*base,*tool);

    // construct the State that should hold the states of the seriel device
    State state(tree);

    /*
     * Now we are ready to construct the serial device
     */
    SerialDevice puma560Device(base,tool,"PUMA560",state);


    // Print end-effector position wrt. base frame
    std::cout << puma560Device.baseTend(state) << std::endl;

    // Change the position of 1 joint
    Q q(6);
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;
    q[4] = 0.0;
    q[5] = M_PI;

    puma560Device.setQ(q,state);

    // Print the new end-effector position wrt. base frame
    std::cout << puma560Device.baseTend(state) << std::endl;

    return 0;
}
