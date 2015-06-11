/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "MessagePacket.hpp"

using namespace rw::common;
using namespace rwlibs::mathematica;

MessagePacket::MessagePacket(const Mathematica::Symbol& symbol, const Mathematica::String& string):
	Packet("MessagePacket",Mathematica::Message),
	_symbol(symbol),
	_string(string)
{
}

MessagePacket::~MessagePacket() {
}

const Mathematica::String& MessagePacket::string() {
	return _string;
}

std::list<rw::common::Ptr<const Mathematica::Expression> > MessagePacket::getArguments() const {
	std::list<rw::common::Ptr<const Mathematica::Expression> > res;
	res.push_back(_symbol.clone());
	res.push_back(_string.clone());
	return res;
}

Mathematica::Expression::Ptr MessagePacket::clone() const {
	return ownedPtr(new MessagePacket(_symbol,_string));
}
