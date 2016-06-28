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

#ifndef RW_COLLISION_PROXIMITYSETUP_HPP
#define RW_COLLISION_PROXIMITYSETUP_HPP

#include "ProximitySetupRule.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/PropertyMap.hpp>

#include <string>
#include <vector>

namespace rw { namespace models { class WorkCell; } }

namespace rw {
namespace proximity {

class CollisionSetup;

/** @addtogroup proximity */
/*@{*/
//! @file rw/proximity/ProximitySetup.hpp
/**
 * @brief Setup for the collision checker
 *
 * The ProximitySetup contains the rules about which frames should be 
 * checked against each other
 */
class ProximitySetup
{
public:
    /**
     * @brief Default constructor for when no excludes are described
     */
    ProximitySetup();

	ProximitySetup(const CollisionSetup& csetup);

    /**
     @brief Constructs ProximitySetup with list of exclusions

     @param exclude [in] pairs to be excluded
     */
	explicit ProximitySetup(const std::vector<ProximitySetupRule>& rules);


	void addProximitySetupRule(const ProximitySetupRule& rule);

	void removeProximitySetupRule(const ProximitySetupRule& rule);

    /**
     * @brief Returns the exclude list
     * @return the exclude list
     */
	const std::vector<ProximitySetupRule>& getProximitySetupRules() const
    {
        return _rules;
    }


    /**
     * @brief Combine setup of this and setup of \b b into this collision setup.
     */
	void merge(const ProximitySetup& setup, const std::string& prefix) {
		BOOST_FOREACH(const ProximitySetupRule& rule, setup.getProximitySetupRules()) {
			std::pair<std::string, std::string> patterns = rule.getPatterns();
			if (prefix != "") {
				patterns.first = prefix+patterns.first;
				patterns.second = prefix+patterns.second;
			}
			addProximitySetupRule(ProximitySetupRule(patterns.first, patterns.second, rule.type()));
		}
	}



	bool useExcludeStaticPairs() const {
		return _useExcludeStaticPairs;
	}

	void setUseExcludeStaticPairs(bool exclude) {
		_useExcludeStaticPairs = exclude;
	}

	bool useIncludeAll() const {
		return _useIncludeAll;
	}

	void setUseIncludeAll(bool includeAll) {
		_useIncludeAll = includeAll;
	}


    static ProximitySetup get(const rw::models::WorkCell& wc);
    static ProximitySetup get(rw::common::Ptr<rw::models::WorkCell> wc);
    static ProximitySetup get(const rw::common::PropertyMap& map);

    static void set(const ProximitySetup& setup, rw::common::Ptr<rw::models::WorkCell> wc);
    static void set(const ProximitySetup& setup, rw::common::PropertyMap& map);

private:
	std::vector<ProximitySetupRule> _rules;

	bool _useIncludeAll;
	bool _useExcludeStaticPairs;
};

/*@}*/
}
} // end namespaces

#endif // end include guard
