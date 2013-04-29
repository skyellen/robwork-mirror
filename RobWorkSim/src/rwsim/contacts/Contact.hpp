/*
 * Contact.hpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#ifndef CONTACT_HPP_
#define CONTACT_HPP_

#include "ContactModel.hpp"
#include <rw/math/Transform3D.hpp>

namespace rwsim {
namespace contacts {

class Contact {
public:
	Contact();
	virtual ~Contact();

	ContactModel::Ptr getModelA() const;
	ContactModel::Ptr getModelB() const;
	rw::math::Transform3D<> aTb() const;
	rw::math::Vector3D<> getPointA() const;
	rw::math::Vector3D<> getPointB() const;
	rw::math::Vector3D<> getNormal() const;
	double getDepth() const;

	void setModelA(ContactModel::Ptr modelA);
	void setModelB(ContactModel::Ptr modelB);
	void setTransform(rw::math::Transform3D<> aTb);
	void setPointA(rw::math::Vector3D<> pointA);
	void getPointB(rw::math::Vector3D<> pointB);
	void getNormal(rw::math::Vector3D<> normal);
	void getDepth(double depth);

	/**
	 * @brief clear all result values
	 */
	virtual void clear();

private:
	ContactModel::Ptr _a, _b;
	rw::math::Transform3D<> _aTb;
	rw::math::Vector3D<> _pointA, _pointB;
	rw::math::Vector3D<> _normal;
	double _depth;
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACT_HPP_ */
