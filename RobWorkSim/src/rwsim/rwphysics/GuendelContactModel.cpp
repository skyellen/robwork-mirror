#include "GuendelContactModel.hpp"


#include <rw/math/Vector3D.hpp>
#include <rw/math/InertiaMatrix.hpp>
//#include <rw/math/Math.hpp>

#include <dynamics/Body.hpp>
#include <dynamics/Contact.hpp>
#include <dynamics/ContactPoint.hpp>

using namespace rw::math;

using namespace rwsim::simulator;

namespace {

    std::pair<double,double>
        impulseCalcFixed(
                      Body& body,
                      double restCoeff,
                      double staticFriction,
                      ContactPoint& contact
                      )
    {
        //double restCoeff = 0.1;
        // get the current relative velocity of point posA
        Vector3D<> relVel = body.getPointVelW( contact.p );

        // calculate the normal component of the relative velocity
        double relVelN = dot( relVel , contact.n );
        // and the tangential component of the relative velocity
        double relVelT = dot( relVel , contact.t );

        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;

        Vector3D<> J = contact.KInv * (-(restCoeff)*relVelN*contact.n - relVel );

        // get the normal and tangential components
        double jn = dot(J,contact.n);
        double jt = dot(J,contact.t);

        std::cout << "* Normal impulse size      : " << jn << std::endl;
        std::cout << "* Tangential impulse size  : " << jt << std::endl;

        // is J outside the friction cone ?
        if( jt > staticFriction*jn){
            // then use sliding friction instead
            double numerator = -(restCoeff+1)*relVelN;
            Vector3D<> nut = contact.n-staticFriction*contact.t;
            double term1 = dot( contact.K*nut, contact.n );
            jn = numerator/term1;
            J = nut*jn;
            jn = dot(J,contact.n);
            jt = dot(J,contact.t);
            std::cout << "* Sliding contact impulse : " << J << std::endl;
        }
        // std::cout << "* Relative velocity is: " << relVel << std::endl;
        // std::cout << "* END OF GUENDEL"<< std::endl;

/*        double tmp = contact.nImpulse;
        contact.nImpulse = std::max(tmp+jn, 0.0);
        jn = contact.nImpulse - tmp;

        float maxPt = staticFrictionCoeff * contact.nImpulse;
        tmp = contact.tImpulse;
        contact.tImpulse = Math::Clamp(tmp+jt, -maxPt, maxPt );
        jt = contact.tImpulse - tmp;

        J = contact.t*jt + n*jn;
*/
        return std::pair<double,double>(jn,jt);
    }

    std::pair<double,double>
        impulseCalc(
                      Body& bodyA,
                      Body& bodyB,
                      double restCoeff,
                      double staticFriction,
                      ContactPoint& contact
                      )
    {
        //double restCoeff = 0.1;
        // get the current relative velocity of point posA
        Vector3D<> pAdot = bodyA.getPointVelW(contact.p);
        Vector3D<> pBdot = bodyB.getPointVelW(contact.p);
        Vector3D<> relVel = pAdot+pBdot;

        // calculate the normal component of the relative velocity
        double relVelN = dot( relVel , contact.n );
        // and the tangential component of the relative velocity
        double relVelT = dot( relVel , contact.t );

        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;

        Vector3D<> J = contact.KInv * (-(restCoeff)*relVelN*contact.n - relVel );

        // get the normal and tangential components
        double jn = dot(J,contact.n);
        double jt = dot(J,contact.t);

        std::cout << "* Normal impulse size      : " << jn << std::endl;
        std::cout << "* Tangential impulse size  : " << jt << std::endl;

        // is J outside the friction cone ?
        if( jt > staticFriction*jn){
            // then use sliding friction instead
            double numerator = -(restCoeff+1)*relVelN;
            Vector3D<> nut = contact.n-staticFriction*contact.t;
            double term1 = dot( contact.K*nut, contact.n );
            jn = numerator/term1;
            J = nut*jn;
            jn = dot(J,contact.n);
            jt = dot(J,contact.t);
            std::cout << "* Sliding contact impulse : " << J << std::endl;
        }
        // std::cout << "* Relative velocity is: " << relVel << std::endl;
        // std::cout << "* END OF GUENDEL"<< std::endl;

        return std::pair<double,double>(jn,jt);
    }

    void preImpulseCalcFixed(
                             Body& body,
                             ContactPoint& contact
                             )
    {
        // compute the tangent direction velocity component
        Vector3D<> relVel = body.getPointVelW( contact.p );
        double relVelN = dot(relVel , contact.n);
        contact.t = relVel - relVelN*contact.n;
        double relVelT = contact.t.norm2();

        // make sure to handle situations where tangential velocity is very small
        if( relVelT < 0.000001 ) {
            contact.t = Vector3D<>(0,0,0);
        } else {
            contact.t /= relVelT;
        }

        // calculate the vector from center of mass (bodyA) to contact point posA
        Vector3D<> ra = contact.p - body.getWTBody().P();

        // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/massinv
        // Skew calculates the cross-product matrix from a vector
        double massInv = body.getInvMass();
        InertiaMatrix<> iInv = body.getInertiaTensorInvW();
        Rotation3D<> skewra = Skew(ra);
        InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
        K(0,0) += massInv;
        K(1,1) += massInv;
        K(2,2) += massInv;

        // now calculate the inverse of K
        contact.K = K;
        contact.KInv = inverse( K );
    }

    void preImpulseCalcImpl(
                        Body& bodyA,
                        Body& bodyB,
                        ContactPoint& contact
                        )
    {
        // compute the tangent direction velocity component
        Vector3D<> pAdot = bodyA.getPointVelW(contact.p);
        Vector3D<> pBdot = bodyB.getPointVelW(contact.p);

        double relVelNA = dot(pAdot,contact.n);
        double relVelNB = dot(pBdot,contact.n);

        double relVelN = relVelNA + relVelNB;

        contact.t = (pAdot+pBdot) - relVelN*contact.n;

        double relVelT = contact.t.norm2();

        double epsilon = 0.0000001;

        // make sure to handle situations where tangential velocity is very small
        if( relVelT < epsilon ) {
            contact.t = Vector3D<>(0,0,0);
        } else {
            contact.t /= relVelT;
        }

        // calculate K_A
        // calculate the vector from center of mass (bodyA) to contact point posA
        Vector3D<> ra = contact.p - bodyA.getWTBody().P();
        // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/massinv
        // Skew calculates the cross-product matrix from a vector
        double massInvA = bodyA.getInvMass();
        InertiaMatrix<> iInvA = bodyA.getInertiaTensorInvW();
        Rotation3D<> skewra = Skew(ra);
        InertiaMatrix<> KA =  (inverse(skewra) * iInvA) * skewra;
        KA(0,0) += massInvA;
        KA(1,1) += massInvA;
        KA(2,2) += massInvA;

        // calculate K_B
        // calculate the vector from center of mass (bodyA) to contact point posA
        Vector3D<> rb = contact.p - bodyB.getWTBody().P();
        // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/massinv
        // Skew calculates the cross-product matrix from a vector
        double massInvB = bodyB.getInvMass();
        InertiaMatrix<> iInvB = bodyB.getInertiaTensorInvW();
        Rotation3D<> skewrb = Skew(rb);
        InertiaMatrix<> KB =  (inverse(skewrb) * iInvB) * skewrb;
        KB(0,0) += massInvB;
        KB(1,1) += massInvB;
        KB(2,2) += massInvB;

        // K is then
        contact.K = KA+KB;
        // now calculate the inverse of K
        contact.KInv = inverse( contact.K );
    }
}

void GuendelContactModel::preImpulseCalc(Contact& contact, ContactPoint& point){
    // calculate aux variables used in the impulse calculation
    Body *bodyA = contact.bodyA;
    Body *bodyB = contact.bodyB;

    // test if one of the bodies are fixed
    if( bodyA->getNodeType()==ConstraintNode::Fixed ){
        preImpulseCalcFixed(*bodyB, point);
    } else if( bodyB->getNodeType()==ConstraintNode::Fixed ){
        preImpulseCalcFixed(*bodyA, point);
    } else {
        // none of the bodies are fixed
        preImpulseCalcImpl(*bodyA, *bodyB, point);
    }
}

void GuendelContactModel::calcCollisionImpulse(Contact& contact,
                                                ContactPoint& point,
                                                double& nimpulse,
                                                double& timpulse)
{
    Body *bodyA = contact.bodyA;
    Body *bodyB = contact.bodyB;
    std::pair<double,double> res(0.0,0.0);
    // test if one of the bodies are fixed
    if( bodyA->getNodeType()==ConstraintNode::Fixed ){
        res = impulseCalcFixed(*bodyB, contact.nColRestCoeff, contact.staticFriction, point);
    } else if( bodyB->getNodeType()==ConstraintNode::Fixed ){
        res = impulseCalcFixed(*bodyA, contact.nColRestCoeff, contact.staticFriction, point);
    } else {
        // none of the bodies are fixed
        res = impulseCalc(*bodyA, *bodyB, contact.nColRestCoeff, contact.staticFriction, point);
    }
    nimpulse = res.first;
    timpulse = res.second;
}

void ContactModel::calcContactImpulse(Contact& contact,
                                              ContactPoint& point,
                                              double& nimpulse,
                                              double& timpulse)
{
    Body *bodyA = contact.bodyA;
    Body *bodyB = contact.bodyB;
    std::pair<double,double> res(0.0,0.0);
    // test if one of the bodies are fixed
    if( bodyA->getNodeType()==ConstraintNode::Fixed ){
        res = impulseCalcFixed(*bodyB, contact.nConRestCoeff, contact.staticFriction, point);
    } else if( bodyB->getNodeType()==ConstraintNode::Fixed ){
        res = impulseCalcFixed(*bodyA, contact.nConRestCoeff, contact.staticFriction, point);
    } else {
        // none of the bodies are fixed
        res = impulseCalc(*bodyA, *bodyB, contact.nConRestCoeff, contact.staticFriction, point);
    }
    nimpulse = res.first;
    timpulse = res.second;
}






