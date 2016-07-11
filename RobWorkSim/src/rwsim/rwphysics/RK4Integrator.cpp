/*
 * RK4Integrator.hpp
#include  *
 *  Created on: 22-10-2008
 *      Author: jimali
 */

#include "RK4Integrator.hpp"

#include <dynamics/RigidBody.hpp>

using namespace rwsim::simulator;
using namespace rw::math;

namespace {
    /**
     * RKStep1 :
            acc1 = force * (h / mass / 2.0);
            vel1 = vel;
            pos += vel1 * (h / 2.0);
            vel += acc1;
     */
    void RKStep1(double h, double massInv,
                 const Vector3D<>& force, const Vector3D<>& torque,
                 ){

        Vector3D<> linAcc = force * massInv * h * 2;

        Vector3D<> tau  = cross( angVel, ITensor*angVel );
        Vector3D<> angAcc = ITensorInv * (torque /*- tau*/);

    }

}

RK4Integrator::RK4Integrator(RigidBody *body):
    _body(body)
{}

void RK4Integrator::updatePosition(double h, rw::kinematics::State& state)
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

void RK4Integrator::updateVelocity(double h, rw::kinematics::State& state)
{

    Vector3D<> linAcc = _body->getForce() * _body->getMassInv();

    // update linear velocity
    rw::math::Vector3D<> linVel  = _body->getLinVel();
    rw::math::Vector3D<> angVel  = _body->getAngVel();
    const rw::math::Vector3D<> &torque = _body->getTorque();

    const InertiaMatrix<> &IBody    = _body->getBodyInertia();
    const InertiaMatrix<> &IBodyInv = _body->getBodyInertiaInv();
    const Transform3D<> &pTb = _body->getPTBody();

    // Calculate the inertia matrix and its inverse
    const InertiaMatrix<> &ITensorInv = _body->getInertiaTensorInv();
    const InertiaMatrix<> &ITensor = _body->getInertiaTensor();

    linVel += linAcc*h;

    // calculate the angular acceleration
    // omega = Iinv * (torque - cross(omega,I*omega))
    rw::math::Vector3D<> tau  = cross( angVel, ITensor*angVel );
    rw::math::Vector3D<> angAcc = ITensorInv * (torque /*- tau*/);

    // update the angular velocity
    angVel += angAcc*h;

    _body->setLinVel(linVel);
    _body->setAngVel(angVel);
}
