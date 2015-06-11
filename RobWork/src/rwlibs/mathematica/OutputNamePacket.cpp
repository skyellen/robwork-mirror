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

#include "OutputNamePacket.hpp"

using namespace rw::common;
using namespace rwlibs::mathematica;

OutputNamePacket::OutputNamePacket(const Mathematica::String& string):
	Packet("OutputNamePacket",Mathematica::OutputName),
	_string(string)
{
}

OutputNamePacket::~OutputNamePacket() {
}

const Mathematica::String& OutputNamePacket::string() {
	return _string;
}

std::list<rw::common::Ptr<const Mathematica::Expression> > OutputNamePacket::getArguments() const {
	return std::list<rw::common::Ptr<const Mathematica::Expression> >(1, _string.clone());
}

Mathematica::Expression::Ptr OutputNamePacket::clone() const {
	return ownedPtr(new OutputNamePacket(_string));
}
