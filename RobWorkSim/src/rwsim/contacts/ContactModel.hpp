/*
 * ContactModel.hpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#ifndef CONTACTMODEL_HPP_
#define CONTACTMODEL_HPP_

#include <rw/proximity/ProximityModel.hpp>

namespace rwsim {
namespace contacts {

class ContactModel: public rw::proximity::ProximityModel {
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<ContactModel> Ptr;

	ContactModel(rw::proximity::ProximityStrategy* pOwner): ProximityModel(pOwner) {};
	virtual ~ContactModel() {};

	virtual std::string getName() = 0;
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTMODEL_HPP_ */
