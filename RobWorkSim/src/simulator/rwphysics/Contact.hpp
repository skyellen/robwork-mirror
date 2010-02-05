#ifndef CONTACT_HPP_
#define CONTACT_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/proximity/DistanceToleranceStrategy.hpp>

#include "RWBody.hpp"
#include <dynamics/ContactPoint.hpp>

namespace dynamics {
	class ContactModel;
}

typedef std::vector<ContactPoint> ContactPointList;

class Contact
{
public:

	Contact(dynamics::ContactModel *model):
		_model(model),
		nColRestCoeff(0.2),
		nConRestCoeff(0.0)
	{}

	virtual ~Contact(){};

	dynamics::ContactModel& getModel(){
		return *_model;
	}

	RWBody *bodyA, *bodyB; // the contacting bodies

	ContactPointList contactPoints;

	double staticFriction; // the static friction of the contact
						   // this is generel for all contact points between b1 and b2

	double nColRestCoeff;

	double nConRestCoeff;

private:
	Contact(){};

	dynamics::ContactModel *_model;
};

#endif /*CONTACT_HPP_*/
