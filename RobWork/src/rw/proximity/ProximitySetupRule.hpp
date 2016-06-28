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

#ifndef RW_COLLISION_PROXIMITYFILTERRULE_HPP
#define RW_COLLISION_PROXIMITYFILTERRULE_HPP

#include <boost/regex.hpp>

namespace rw {
namespace proximity {

/** @addtogroup proximity */
/*@{*/
//! @file rw/proximity/ProximitySetupRule.hpp

	/**
	 * @brief Rule specifying include/exclude of frame pairs
	 *
	 * The rule has two patterns, pattern A and pattern B, to which frames can be matched. A pattern
	 * could contain a fully specified frame name such as "Table". It can also include 
	 * wild card characters such a "Robot.*" or regular expressions.
	 */
	class ProximitySetupRule {
	public:

		/** @brief Include and Exclude identifiers */
		enum RuleType {INCLUDE_RULE = 1, EXCLUDE_RULE}; 

		/** 
		 * @brief Constructs rule with patternA and patternB and type
		 *
		 * @param patternA [in] Pattern identifying first frame in rule
		 * @param patternB [in] Pattern identifying second frame in rule
		 */
		ProximitySetupRule(const std::string& patternA, const std::string& patternB, RuleType type);

		/**
		 * @brief Returns the string patterns used to match
		 */
		std::pair<std::string, std::string> getPatterns() const {
			return _patterns;
		}

		/**
		 * @brief Returns the type of rule
		 */
		RuleType type() const {
			return _type;
		}


		/**
		 * @brief Check whether \b str1 and \b str2 matches the pattern.
		 *
		 * Success is defined if the first pattern matches \b str1 and the second
		 * matches \b str2 or the first matches \b str2 and the second \b str1.
		 */
		bool match(const std::string& str1, const std::string& str2) const {
			return (matchPatternA(str1) && matchPatternB(str2)) ||
				   (matchPatternA(str2) && matchPatternB(str1));

		}
	
		/**
		 * @brief Check whether \b pair matches the patterns.
		 *
		 * Success is defined if the first pattern matches \b pair.first and the second
		 * matches \b pair.second or the first matches \b pair.second and the second \b pair.first.
		 */
		bool match(std::pair<std::string,std::string>& pair) const {
			return match(pair.first, pair.second);
		}

		/**
		 * @brief Check whether \b name matches one of the patterns
		 */
		bool matchOne(const std::string& name) const {
			if (boost::regex_match(name, _regex1) || boost::regex_match(name, _regex2)) {
				return true;
			}
			return false;
		}

		/**
		 * @brief Check whether \b str matches pattern A
		 */
		bool matchPatternA(const std::string& str) const {
			return boost::regex_match(str, _regex1);
		}


		/**
		 * @brief Check whether \b name matches pattern B
		 */
		bool matchPatternB(const std::string& str) const {
			return boost::regex_match(str, _regex2);
		}


		 
		/**
		 * @brief ostream operator formatting the rule for easy reading
		 */
		friend std::ostream& operator<<(std::ostream& s, const ProximitySetupRule& r) {
			if (r._type == ProximitySetupRule::INCLUDE_RULE)
				s<<"Include: ";
			else
				s<<"Exclude: ";
			s<<r._patterns.first<<" <=> "<<r._patterns.second;
			return s;
		}

		/**
		 * @brief Compares if two rules are the same
		 */
		bool operator==(const ProximitySetupRule& p) const { return _patterns == p._patterns && _type == p._type; }


		/**
		 * @brief Make an exclude rule for the patterns
		 */
		static ProximitySetupRule makeExclude(const std::pair<std::string, std::string>& patterns) {
			return ProximitySetupRule(patterns.first, patterns.second, EXCLUDE_RULE);
		}

		/**
		 * @brief Make an exclude rule for the patterns
		 */
		static ProximitySetupRule makeExclude(const std::string& patternA, const std::string& patternB) {
			return ProximitySetupRule(patternA, patternB, EXCLUDE_RULE);
		}

		/**
		 * @brief Make an include rule for the patterns
		 */
		static ProximitySetupRule makeInclude(const std::pair<std::string, std::string>& patterns) {
			return ProximitySetupRule(patterns.first, patterns.second, INCLUDE_RULE);
		}

		/**
		 * @brief Make an include rule for the patterns
		 */
		static ProximitySetupRule makeInclude(const std::string& patternA, const std::string& patternB) {
			return ProximitySetupRule(patternA, patternB, INCLUDE_RULE);
		}


	private:
		std::pair<std::string, std::string> _patterns;	
		boost::regex _regex1;
		boost::regex _regex2;
		RuleType _type;
	};


/*@}*/
}
} // end namespaces

#endif // end include guard
