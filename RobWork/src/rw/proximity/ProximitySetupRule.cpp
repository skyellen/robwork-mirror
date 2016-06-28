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


#include "ProximitySetupRule.hpp"
#include <rw/common/StringUtil.hpp>

using namespace rw::common;
using namespace rw::proximity;



namespace {
	boost::regex convertToBoostRegEx(const std::string& ex) {
		if (ex.size() > 5 && ex.substr(0, 5) == "REGEX") {
			return boost::regex(ex.substr(5, ex.size()-5));
		}  else {
			return boost::regex(StringUtil::patternToRegEx(ex));
		}
	}
}



ProximitySetupRule::ProximitySetupRule(const std::string& patternA, const std::string& patternB, RuleType type):
	_patterns(patternA, patternB),
	_regex1(convertToBoostRegEx(patternA)),
	_regex2(convertToBoostRegEx(patternB)),
	_type(type)
{

}
