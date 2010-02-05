#ifndef CONTACTPOINT_HPP_
#define CONTACTPOINT_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/InertiaMatrix.hpp>

class ContactPoint
{
public:

    ContactPoint():
        isFirstContact(true),
        nImpulse(0.0),
	    tImpulse(0.0),
	    dist(100),
	    penetrationA(0.0),
	    penetrationB(0.0),
	    penetration(0.0),
	    K(1.0,1.0,1.0),
	    KInv(1.0,1.0,1.0)
	{};

	virtual ~ContactPoint(){};

	// following is the default and minimum requirements for a
	// contact point description

    bool isFirstContact; // true if first contact

    double nImpulse, // normal impulse
           tImpulse; // tangential impulse

    double dist; // distance between pA and pB
    double bias;
    double penetrationA; // the penetration depth of A into B
    double penetrationB; // the penetration depth of B into A

    double nForce,
    	   tForce;

    // the penetration of the contact point
    double penetration;

    rw::math::Vector3D<> p, // the contact position
                         n, // the contact normal
                         t; // the tangential unit velocity

    rw::math::Vector3D<> pA,    // position of contact point on core of A
                         pAInit,// position pA recorded on first contact
                         pB,    // position of contact point on core of B
                         pBInit;// position pB recorded on first contact

    // these variables are not generel and therefore should not be here
    rw::math::InertiaMatrix<> K,KInv; //

    // user specific data
    void *userdata;


};

#endif /*CONTACTPOINT_HPP_*/
