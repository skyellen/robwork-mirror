/*
 * ContactDetectorData.hpp
 *
 *  Created on: 19/04/2013
 *      Author: thomas
 */

#ifndef CONTACTDETECTORDATA_HPP_
#define CONTACTDETECTORDATA_HPP_

#include "ContactStrategyData.hpp"

namespace rwsim {
namespace contacts {

class ContactDetectorData {
public:
	ContactDetectorData();
	virtual ~ContactDetectorData();

	std::vector<Contact> getContacts();
	void setContacts(const std::vector<Contact> &contacts);

	ContactStrategyData getStrategyData();
	void setStrategyData(const ContactStrategyData &data);

private:
	std::vector<Contact> _contacts;
	ContactStrategyData _stratData;
};

} /* namespace contacts */
} /* namespace rwsim */
#endif /* CONTACTDETECTORDATA_HPP_ */
