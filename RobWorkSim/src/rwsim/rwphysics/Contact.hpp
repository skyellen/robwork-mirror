#ifndef RWSIM_SIMULATOR_CONTACT_HPP_
#define RWSIM_SIMULATOR_CONTACT_HPP_

#include "RWBody.hpp"
#include <rwsim/dynamics/ContactPoint.hpp>

namespace rwsim {
namespace simulator {

	class ContactModel;

	typedef std::vector<rwsim::dynamics::ContactPoint> ContactPointList;

	class Contact
	{
	public:

		Contact(ContactModel *model):
            nColRestCoeff(0.2),
            nConRestCoeff(0.0),
		    _model(model)
		{}

		virtual ~Contact(){};

		ContactModel& getModel(){
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

		ContactModel *_model;
	};

}
}

#endif /*CONTACT_HPP_*/
