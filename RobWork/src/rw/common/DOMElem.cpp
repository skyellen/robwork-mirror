/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "DOMElem.hpp"

#include <rw/common/StringUtil.hpp>
#include <boost/lexical_cast.hpp>

using namespace rw::common;

bool DOMElem::getValueAsBool(){
	std::string val = getValue();
	std::string valu = rw::common::StringUtil::toUpper(val);
	if(valu=="TRUE") return true;
	if(valu=="FALSE") return true;
	return boost::lexical_cast<bool>(val);
}

void DOMElem::setValue(bool val){
	setValue( boost::lexical_cast<std::string>(val) );
}

void DOMElem::setValue(int val){
	setValue( boost::lexical_cast<std::string>(val) );
}

void DOMElem::setValue(double val){
	setValue( boost::lexical_cast<std::string>(val) );
}

