/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ProximityStrategyPlugin.hpp"

#include <RobWorkConfig.hpp>

#ifdef RW_HAVE_BULLET
#include "ProximityStrategyBullet.hpp"
#endif

#ifdef RW_HAVE_FCL
#include "ProximityStrategyFCL.hpp"
#endif

#ifdef RW_HAVE_PQP
#include "ProximityStrategyPQP.hpp"
#endif

#ifdef RW_HAVE_YAOBI
#include "ProximityStrategyYaobi.hpp"
#endif

using namespace rw::common;
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;

RW_ADD_PLUGIN(ProximityStrategyPlugin)

ProximityStrategyPlugin::ProximityStrategyPlugin():
	Plugin("ProximityStrategyPlugin", "ProximityStrategyPlugin", "1.0")
{
}

ProximityStrategyPlugin::~ProximityStrategyPlugin() {
}

std::vector<Extension::Descriptor> ProximityStrategyPlugin::getExtensionDescriptors() {
    std::vector<Extension::Descriptor> exts;

#ifdef RW_HAVE_BULLET
    {
    	static const std::string stratID = "Bullet";
    	exts.push_back(Extension::Descriptor("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionToleranceStrategy"+stratID,"rw.proximity.CollisionToleranceStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("DistanceMultiStrategy"+stratID,"rw.proximity.DistanceMultiStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    }
#endif

#ifdef RW_HAVE_FCL
    {
    	static const std::string stratID = "FCL";
    	exts.push_back(Extension::Descriptor("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    }
#endif

#ifdef RW_HAVE_PQP
    {
    	static const std::string stratID = "PQP";
    	exts.push_back(Extension::Descriptor("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionToleranceStrategy"+stratID,"rw.proximity.CollisionToleranceStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("DistanceMultiStrategy"+stratID,"rw.proximity.DistanceMultiStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    }
#endif

#ifdef RW_HAVE_YAOBI
    {
    	static const std::string stratID = "Yaobi";
    	exts.push_back(Extension::Descriptor("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    	exts.push_back(Extension::Descriptor("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy"));
    	exts.back().getProperties().set<std::string>("strategyID", stratID);
    }
#endif

    return exts;
}

Extension::Ptr ProximityStrategyPlugin::makeExtension(const std::string& id) {
	// Important: When Extensions are created from AnyPtr objects, a specific strategy object MUST be cast to the correct base type before creating the AnyPtr.
	// - Due to multiple inheritance, the memory layout of the specific strategy is unknown to the loading program. Casting from a void* will then cause nasty vtable failures.

#ifdef RW_HAVE_BULLET
	{
		static const ProximityStrategyBullet::Ptr strat = ownedPtr(new ProximityStrategyBullet());
		static const std::string stratID = "Bullet";
		if (id == "ProximityStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy", this, strat.cast<ProximityStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy", this, strat.cast<CollisionStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionToleranceStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionToleranceStrategy"+stratID,"rw.proximity.CollisionToleranceStrategy", this, strat.cast<CollisionToleranceStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "DistanceStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy", this, strat.cast<DistanceStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "DistanceMultiStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("DistanceMultiStrategy"+stratID,"rw.proximity.DistanceMultiStrategy", this, strat.cast<DistanceMultiStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		}
	}
#endif

#ifdef RW_HAVE_FCL
	{
		static const ProximityStrategyFCL::Ptr strat = ownedPtr(new ProximityStrategyFCL());
		static const std::string stratID = "FCL";
		if (id == "ProximityStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy", this, strat.cast<ProximityStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy", this, strat.cast<CollisionStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "DistanceStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy", this, strat.cast<DistanceStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		}
	}
#endif

#ifdef RW_HAVE_PQP
	{
		static const ProximityStrategyPQP::Ptr strat = ownedPtr(new ProximityStrategyPQP());
		static const std::string stratID = "PQP";
		if (id == "ProximityStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy", this, strat.cast<ProximityStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy", this, strat.cast<CollisionStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionToleranceStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionToleranceStrategy"+stratID,"rw.proximity.CollisionToleranceStrategy", this, strat.cast<CollisionToleranceStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "DistanceStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("DistanceStrategy"+stratID,"rw.proximity.DistanceStrategy", this, strat.cast<DistanceStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "DistanceMultiStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("DistanceMultiStrategy"+stratID,"rw.proximity.DistanceMultiStrategy", this, strat.cast<DistanceMultiStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		}
	}
#endif

#ifdef RW_HAVE_YAOBI
	{
		static const ProximityStrategyYaobi::Ptr strat = ownedPtr(new ProximityStrategyYaobi());
		static const std::string stratID = "Yaobi";
		if (id == "ProximityStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("ProximityStrategy"+stratID,"rw.proximity.ProximityStrategy", this, strat.cast<ProximityStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		} else if (id == "CollisionStrategy"+stratID) {
			const Extension::Ptr extension = ownedPtr(new Extension("CollisionStrategy"+stratID,"rw.proximity.CollisionStrategy", this, strat.cast<CollisionStrategy>()));
			extension->getProperties().set<std::string>("strategyID", stratID);
			return extension;
		}
	}
#endif

	return NULL;
}

void ProximityStrategyPlugin::registerPlugin() {
	ExtensionRegistry::getInstance()->registerExtensions(ownedPtr(new ProximityStrategyPlugin()));
}
