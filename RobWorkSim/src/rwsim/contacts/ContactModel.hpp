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
	ContactModel(rw::proximity::ProximityStrategy* pOwner);
	virtual ~ContactModel();
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTMODEL_HPP_ */
