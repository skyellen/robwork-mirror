/*
 * ContactDetectorData.cpp
 *
 *  Created on: 19/04/2013
 *      Author: thomas
 */

#include "ContactDetectorData.hpp"

using namespace rwsim::contacts;

ContactDetectorData::ContactDetectorData()
{
}

ContactDetectorData::~ContactDetectorData()
{
}

std::vector<Contact> ContactDetectorData::getContacts() {
	return _contacts;
}

void ContactDetectorData::setContacts(const std::vector<Contact> &contacts) {
	_contacts = contacts;
}

ContactStrategyData ContactDetectorData::getStrategyData() {
	return _stratData;
}

void ContactDetectorData::setStrategyData(const ContactStrategyData &data) {
	_stratData = data;
}
