/*
 * ContactModel.cpp
 *
 *  Created on: 18/04/2013
 *      Author: thomas
 */

#include "ContactModel.hpp"

using namespace rw::proximity;
using namespace rwsim::contacts;

ContactModel::ContactModel(ProximityStrategy* pOwner):
	ProximityModel(pOwner)
{
}

ContactModel::~ContactModel() {
}
