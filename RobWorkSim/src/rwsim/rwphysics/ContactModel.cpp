#include "ContactModel.hpp"

#include <rw/math/Vector3D.hpp>
//#include <rw/math/InertiaMatrix.hpp>

#include "RWBody.hpp"
#include "Contact.hpp"
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/FixedLink.hpp>

#include "ContactModelFactory.hpp"

using namespace rw::math;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;

#define DYNAMIC_TEST

namespace {

    std::pair<double,double>
        impulseCalcFixed(
                      const Vector3D<> &pAdot,
                      const Vector3D<> &pBdot,
                      double restCoeff,
                      double staticFriction,
                      ContactPoint& contact
                      )
    {
        // get the current relative velocity of point posA
        Vector3D<> relVel = pBdot-pAdot;

        // calculate the normal component of the relative velocity
        // and the tangential component of the relative velocity
        double relVelN = dot( relVel , contact.n );
        //double relVelT = dot( relVel , contact.t );

#ifdef DYNAMIC_TEST
        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* contact.n  : " << contact.n << std::endl;
        //std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;
#endif
        //Vector3D<> J = contact.KInv * (-(restCoeff)*relVelN*contact.n - relVel );
        // if restCoef is zero
        double uRelN = -(restCoeff)*relVelN - contact.bias;
        Vector3D<> J = contact.KInv * ( uRelN*contact.n - relVel);
        //Vector3D<> J = contact.KInv * (-(restCoeff)*relVelN*contact.n - relVel );


        // get the normal and tangential components
        double jn = dot(J,contact.n);
        double jt = dot(J,contact.t);

#ifdef DYNAMIC_TEST
        std::cout << "* Normal impulse size      : " << jn << std::endl;
        std::cout << "* Tangential impulse size  : " << jt << std::endl;
#endif
        // If the bodies are moving away from each other then dont use SLiding
        if( relVelN > 0 )
        	return std::pair<double,double>(jn,jt);


        // is J outside the friction cone ?
        if( fabs(jt) > staticFriction*jn ){
            // then use sliding friction instead
            double numerator = -(restCoeff+1)*relVelN-contact.bias;
            Vector3D<> nut = contact.n-staticFriction*contact.t;
            double term1 = dot( contact.n, contact.K*nut );
            jn = numerator/term1;
            J = nut*jn;
            jn = dot(J,contact.n);
            jt = dot(J,contact.t);
        }
        return std::pair<double,double>(jn,jt);
    }

	std::pair<double,double>
    impulseCalcFixed(RWBody& bodyA, RWBody& bodyB, double cu, double mu, ContactPoint& c) {
    	Vector3D<> pAdot = bodyA.getPointVelW(c.p);
    	Vector3D<> pBdot = bodyB.getPointVelW(c.p);
    	return impulseCalcFixed(pAdot, pBdot, cu, mu, c);
    }

	std::pair<double,double>
    impulseCalcRigidFixed( RWBody& bodyA, double cu, double mu, ContactPoint& c ) {
    	Vector3D<> pAdot = bodyA.getPointVelW(c.p);
    	return impulseCalcFixed(pAdot, Vector3D<>(0,0,0), cu, mu, c);
    }

	std::pair<double,double>
    impulseCalcFixedRigid( RWBody& bodyB, double cu, double mu, ContactPoint& c ) {
    	Vector3D<> pBdot = bodyB.getPointVelW(c.p);
    	return impulseCalcFixed(Vector3D<>(0,0,0), pBdot, cu, mu, c);
    }

    std::pair<double,double>
        impulseCalc(
        		RWBody& bodyA,
        		RWBody& bodyB,
                      double restCoeff,
                      double staticFriction,
                      ContactPoint& contact
                      )
    {
        RW_THROW("DONT GO HERE");
        	//double restCoeff = 0.1;
        // get the current relative velocity of point posA
        Vector3D<> pAdot = bodyA.getPointVelW(contact.p);
        Vector3D<> pBdot = bodyB.getPointVelW(contact.p);
        Vector3D<> relVel = pBdot-pAdot;

        // calculate the normal component of the relative velocity
        // and the tangential component of the relative velocity
        double relVelN = dot( relVel , contact.n );
        double relVelT = dot( relVel , contact.t );

#ifdef DYNAMIC_TEST
        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;
#endif

        Vector3D<> J = contact.KInv * (-(restCoeff)*relVelN*contact.n - relVel );

        // get the normal and tangential components
        double jn = dot(J,contact.n);
        double jt = dot(J,contact.t);
        if( relVelN>0 )
        	return std::pair<double,double>(jn,jt);

#ifdef DYNAMIC_TEST
        std::cout << "* Normal impulse size      : " << jn << std::endl;
        std::cout << "* Tangential impulse size  : " << jt << std::endl;
#endif
        // is J outside the friction cone ?
        if( fabs(jt) > staticFriction*jn ){
            // then use sliding friction instead
            double numerator = -(restCoeff+1)*relVelN-contact.bias;
            Vector3D<> nut = contact.n-staticFriction*contact.t;
            double term1 = dot( contact.K*nut, contact.n );
            jn = numerator/term1;
            J = nut*jn;
            jn = dot(J,contact.n);
            jt = dot(J,contact.t);
        }
        return std::pair<double,double>(jn,jt);
    }

    void preImpulseCalcFixed(RWBody& body,
                             const Vector3D<> &pAdot,
                             const Vector3D<> &pBdot,
                             ContactPoint& contact,
                             double dtInv,
                             ContactModelFactory *fact
                             )
    {
        // compute the tangent direction velocity component
        std::cout << "Pre impulse calc" << std::endl;
        Vector3D<> relVel = pBdot-pAdot;
        double relVelN = dot(relVel, contact.n);
        contact.t = relVel - relVelN*contact.n;
        double relVelT = contact.t.norm2();

#ifdef DYNAMIC_TEST
        std::cout << "* relVel : " << relVel << std::endl;
        std::cout << "* pAdot : " << pAdot << std::endl;
        std::cout << "* pBdot : " << pBdot << std::endl;
        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;
#endif
        // make sure to handle situations where tangential velocity is very small
        if( fabs( relVelT ) < 0.000001 ) {
            contact.t = Vector3D<>(0,0,0);
        } else {
            contact.t /= relVelT;
        }
#ifdef DYNAMIC_TEST
        std::cout << "* RELVEL NORMAL  SIZE  : " << relVelN << std::endl;
        std::cout << "* RELVEL TANGENT SIZE  : " << relVelT << std::endl;
#endif
        contact.K = body.getEffectiveMassW( contact.p );
        contact.KInv = inverse( contact.K );

        // calculate the vector from center of mass (bodyA) to contact point posA
/*        Vector3D<> ra = contact.p - body.getWTBody().P();
        //std::cout << "ra" << std::endl;
        // K = trans( Skew(ra) ) * iInv * Skew(ra) + identity3x3*1/mass
        // Skew calculates the cross-product matrix from a vector
        double massInv = body.getInvMass();
        //std::cout << "massInv " << massInv << std::endl;
        InertiaMatrix<> iInv = body.getInertiaTensorInvW();
        Rotation3D<> skewra = Skew(ra);
        InertiaMatrix<> K =  (inverse(skewra) * iInv) * skewra;
        K(0,0) += massInv;
        K(1,1) += massInv;
        K(2,2) += massInv;
        //std::cout << "K" << K << std::endl;
        // now calculate the inverse of K
        contact.K = K;
        contact.KInv = inverse( K );*/

        // Calculate baumgarten stabilization, linear position error feedback
        //double kBiasFactor = 0.2;
        std::cout << "4" << std::endl;
        //double slopSize = (fact->getTouchDist() - fact->getPenetrationDist())/2.0;
        std::cout << "5" << std::endl;
        //double biasLayer = fact->getTouchDist()-slopSize;
        std::cout << "6" << std::endl;
        //contact.bias = kBiasFactor * dtInv * std::max(0.0, biasLayer-contact.dist );
        contact.bias = 0;
#ifdef DYNAMIC_TEST
        std::cout << "Dist: " << contact.dist << std::endl;
        std::cout << "DtInv: " << dtInv << std::endl;
        //std::cout << "BiasLayer: " << biasLayer << std::endl;
        //std::cout << "Pos: " << (biasLayer-contact.dist) << std::endl;
        //std::cout << "SoftPen: " << slopSize << std::endl;
        std::cout << "Bias: " << contact.bias << std::endl;
#endif

    }

    void preImpulseCalcRigidLink(RWBody& body, RWBody& link, ContactPoint& contact,
    							double dtInv, ContactModelFactory *fact){
    	Vector3D<> pAdot = body.getPointVelW( contact.p );
    	Vector3D<> pBdot = link.getPointVelW( contact.p );
    	preImpulseCalcFixed(body, pAdot, pBdot, contact, dtInv, fact);
    }

    void preImpulseCalcLinkRigid(RWBody& link, RWBody& body, ContactPoint& contact,
    							double dtInv, ContactModelFactory *fact){
    	Vector3D<> pAdot = link.getPointVelW( contact.p );
    	Vector3D<> pBdot = body.getPointVelW( contact.p );
    	preImpulseCalcFixed(body, pAdot, pBdot, contact, dtInv, fact);
    }

    void preImpulseCalcRigidFixed(RWBody& body, ContactPoint& contact,
    							  double dtInv, ContactModelFactory *fact){
    	Vector3D<> pAdot = body.getPointVelW( contact.p );
    	preImpulseCalcFixed(body, pAdot, Vector3D<>(0,0,0), contact, dtInv, fact);
    }

    void preImpulseCalcFixedRigid(RWBody& body, ContactPoint& contact,
    							  double dtInv, ContactModelFactory *fact){

        std::cout << "Velocity" << std::endl;
        std::cout << body.getLinVelW() << std::endl;
        std::cout << body.getAngVelW() << std::endl;

        Vector3D<> pBdot = body.getPointVelW( contact.p );
    	preImpulseCalcFixed(body, Vector3D<>(0,0,0), pBdot, contact, dtInv, fact);
    }

    void preImpulseCalcImpl(
    		RWBody& bodyA,
                        RWBody& bodyB,
                        ContactPoint& contact,
                        double dtInv
                        )
    {
        // compute the tangent direction velocity component
        Vector3D<> pAdot = bodyA.getPointVelW(contact.p);
        Vector3D<> pBdot = bodyB.getPointVelW(contact.p);

        //double relVelNA = dot(pAdot, contact.n);
        //double relVelNB = dot(pBdot, contact.n);
        Vector3D<> relVel = pBdot - pAdot;
        double relVelN = dot(relVel, contact.n);
        contact.t = relVel - relVelN*contact.n;
        double relVelT = contact.t.norm2();

        double epsilon = 0.0000001;

        // make sure to handle situations where tangential velocity is very small
        if( fabs(relVelT) < epsilon ) {
            contact.t = Vector3D<>(0,0,0);
        } else {
            contact.t /= relVelT;
        }
        /*
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
        */
        /*
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

        */

        contact.K = bodyA.getEffectiveMassW( contact.p ) +
        			bodyB.getEffectiveMassW( contact.p );
        contact.KInv = inverse( contact.K );
    }
}

ContactModel::ContactModel(ConstraintNode &bodyA, ConstraintNode &bodyB, ContactModelFactory *fact):
	_factory(fact)
{
    switch( bodyA.getNodeType() ){
    case (ConstraintNode::Fixed) :
        // FixedRigid, FixedLink
        if( bodyB.getNodeType()==ConstraintNode::Link ){
            _type = FixedLink;
        } else {
            _type = FixedRigid;
        }
        break;
    case (ConstraintNode::Rigid):
        // RigidFixed, RigidLink, RigidRigid
        if( bodyB.getNodeType()==ConstraintNode::Fixed ){
            _type = RigidFixed;
        } else if( bodyB.getNodeType()==ConstraintNode::Link ){
            _type = RigidLink;
        } else {
            _type = RigidRigid;
        }
        break;
    case ( ConstraintNode::Link ):
        // RigidFixed, RigidLink, RigidRigid
        if( bodyB.getNodeType()==ConstraintNode::Fixed ){
            _type = LinkFixed;
        } else if( bodyB.getNodeType()==ConstraintNode::Link ){
            _type = LinkLink;
        } else {
            _type = LinkRigid;
        }
        break;
    default:
        _type = Unknown;
    }
}

void ContactModel::addForce(Contact& contact,
                                ContactPoint& point,
                                double nforce,
                                double tforce)
        {
			RWBody *bodyA = contact.bodyA;
			RWBody *bodyB = contact.bodyB;
            rw::math::Vector3D<> F = /*point.t*tforce +*/ point.n*nforce;
            //std::cout << "Adding force F: " << F << std::endl;
            switch(_type){
            case(LinkRigid):
            	((dynamics::FixedLink*)bodyA)->addForceWToPosW( -F, point.p);
            	//((RWBody*)bodyB)->addForceWToPosW( F, point.p);
            	break;            case(FixedRigid):
                //((RWBody*)bodyB)->addForceWToPosW( F, point.p);
            	break;
            case(RigidLink):
            	//((RWBody*)bodyA)->addForceWToPosW( -F, point.p);
            	((dynamics::FixedLink*)bodyB)->addForceWToPosW( F, point.p);
            	break;
            case(RigidFixed):
                //((RWBody*)bodyA)->addForceWToPosW( -F, point.p);
            	break;
            case(RigidRigid):
                //((RWBody*)bodyA)->addForceWToPosW(-F, point.p);
                //((RWBody*)bodyB)->addForceWToPosW( F, point.p);
                break;
            case(FixedLink): // not supported
                ((dynamics::FixedLink*)bodyB)->addForceWToPosW( F, point.p);
            	break;
            case(LinkFixed):
                //((dynamics::FixedLink*)bodyA)->addForceWToPosW(-F, point.p);
            	break;
            case(LinkLink):
                ((dynamics::FixedLink*)bodyA)->addForceWToPosW(-F, point.p);
                ((dynamics::FixedLink*)bodyB)->addForceWToPosW( F, point.p);
            	break;
            case(Unknown):
            default:
                RW_WARN( "addForce: unsupported contact type" );
            }
        }

void ContactModel::preImpulseCalc(Contact& contact, ContactPoint& point, double dtInv){
    // calculate aux variables used in the impulse calculation
	RWBody *bodyA = contact.bodyA;
	RWBody *bodyB = contact.bodyB;
    if(bodyA == NULL || bodyB==NULL){
        RW_WARN("Invalid body!");
        return;
    }

    switch(_type){
    case(LinkRigid):
    	std::cout << " LR " << std::endl;
    	preImpulseCalcLinkRigid(*bodyA, *bodyB, point, dtInv, _factory);
    	break;
    case(FixedRigid):
    	std::cout << " FR " << std::endl;
        preImpulseCalcFixedRigid(*bodyB, point, dtInv, _factory);
    	break;
    case(RigidLink):
    	std::cout << " RL " << std::endl;
    	preImpulseCalcRigidLink(*bodyA, *bodyB, point, dtInv, _factory);
    	break;
    case(RigidFixed):
    	std::cout << " RF " << std::endl;
        preImpulseCalcRigidFixed(*bodyA, point, dtInv, _factory);
    	break;
    case(RigidRigid):
    	std::cout << " RR " << std::endl;
        preImpulseCalcImpl(*(RWBody*)bodyA, *(RWBody*)bodyB, point, dtInv);
    	break;
    case(FixedLink):
    case(LinkFixed):
    case(LinkLink):
    	break;
    case(Unknown):
    default:
    	RW_WARN("unsupported contact type!!");
    }
}

void ContactModel::calcContactForce(Contact& contact,
                                ContactPoint& point,
                                double& nforce,
                                double& tforce)
{
	// calculate a force that is dependant on position alone
	double shellThickness = 0.0001;
	double pen = std::max( shellThickness -point.dist, 0.0);
	if( pen == 0.0 ){
		// no penetration exist
		nforce = 0.0;
		tforce = 0.0;
		return;
	}
	double springConst = 100000.0;

	nforce = springConst * pen;
	tforce = 0.0;
}



void ContactModel::calcCollisionImpulse(Contact& contact,
                                                ContactPoint& point,
                                                double& nimpulse,
                                                double& timpulse,
                                                int iter)
{
	RWBody *bodyA = contact.bodyA;
	RWBody *bodyB = contact.bodyB;
    std::pair<double,double> res(0.0,0.0);
    // test if one of the bodies are fixed
    double cu = contact.nColRestCoeff;

    if(iter<4){
    	cu = (iter*1/4)-1;
    }
    cu = 0;
    double mu = contact.staticFriction;
    switch(_type){
    case(LinkRigid):
        res = impulseCalcFixed(*bodyA, *bodyB, cu, mu , point);
    	break;
    case(FixedRigid):
        res = impulseCalcFixedRigid(*bodyB, cu, mu, point);
        break;
    case(RigidLink):
        res = impulseCalcFixed(*bodyA, *bodyB, cu, mu, point);
    	break;
    case(RigidFixed):
        res = impulseCalcRigidFixed(*bodyA, cu, mu, point);
        break;
    case(RigidRigid):
        res = impulseCalc(*bodyA, *bodyB, cu, mu, point);
        break;
    case(LinkLink):
    case(LinkFixed):
    case(FixedLink):
    	res.first = 1000;
    	break;
    case(Unknown):
    default:
        RW_WARN("unsupported contact type!!");
    }
    nimpulse = res.first;
    timpulse = res.second;
}

void ContactModel::calcContactImpulse(Contact& contact,
                                      ContactPoint& point,
                                      double& nimpulse,
                                      double& timpulse)
{
	return;
}

void ContactModel::addImpulse(Contact& contact,
                        ContactPoint& point,
                        double nimpulse,
                        double timpulse)
{
	RWBody *bodyA = contact.bodyA;
	RWBody *bodyB = contact.bodyB;

	// calculate the impulse vector
    rw::math::Vector3D<> J = point.t*timpulse + point.n*nimpulse;

    bodyA->addImpulseWToPosW( -J, point.p);
    bodyB->addImpulseWToPosW( J, point.p);
}

void ContactModel::updateVelocity(Contact& contact){
	RWBody *bodyA = contact.bodyA;
	RWBody *bodyB = contact.bodyB;
    bodyA->updateImpulse();
    bodyB->updateImpulse();
}


