#include "EulerIntegrator.hpp"

#include "RWBody.hpp"

using namespace rwsim::simulator;
using namespace rw::math;

EulerIntegrator::EulerIntegrator(RWBody *body):
    _body(body)
{}

void EulerIntegrator::updatePosition(double h, rw::kinematics::State& state)
{
    Transform3D<> pTb = _body->getPTBody();

    // update position
    rw::math::Vector3D<> &pos = pTb.P();
    rw::math::Rotation3D<> &rot = pTb.R();
    rw::math::Vector3D<> linVel = _body->getLinVel();
    rw::math::Vector3D<> angVel = _body->getAngVel();

    pos += linVel*h;

    // update orientation
    rw::math::EAA<> eaa( angVel(0)*h, angVel(1)*h, angVel(2)*h );
    rot =  eaa.toRotation3D() * rot;

    _body->setPTBody( Transform3D<>(pos, rot), state);
}

void EulerIntegrator::updateVelocity(double h, rw::kinematics::State& state)
{

    Vector3D<> linAcc = _body->getForce() * _body->getMassInv();

    // update linear velocity
    rw::math::Vector3D<> linVel  = _body->getLinVel();
    rw::math::Vector3D<> angVel  = _body->getAngVel();
    const rw::math::Vector3D<> &torque = _body->getTorque();

    //const InertiaMatrix<> &IBody    = _body->getBodyInertia();
    //const InertiaMatrix<> &IBodyInv = _body->getBodyInertiaInv();
    //const Transform3D<> &pTb = _body->getPTBody();

    // Calculate the inertia matrix and its inverse
    const InertiaMatrix<> &ITensorInv = _body->getInertiaTensorInv();
    //const InertiaMatrix<> &ITensor = _body->getInertiaTensor();

    // we add a bit of dampning to the velocity
    linVel += linAcc*h*0.95;

    // calculate the angular acceleration
    // omega = Iinv * (torque - cross(omega,I*omega))
    //rw::math::Vector3D<> tau  = cross( angVel, ITensor*angVel );
    rw::math::Vector3D<> angAcc = ITensorInv * (torque /*- tau*/);

    // update the angular velocity
    angVel += angAcc*h;

    _body->setLinVel(linVel);
    _body->setAngVel(angVel);
}
