/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTBROADPHASE_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTBROADPHASE_HPP_

/**
 * @file TNTBroadPhase.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTBroadPhase
 */

#include <rw/common/Ptr.hpp>

#include <list>
#include <map>
#include <set>
#include <vector>

// Forward declarations
namespace rw { namespace geometry { class Geometry; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace kinematics { template <class T> class FrameMap; } }
namespace rw { namespace kinematics { class State; } }
namespace rw { namespace models { class Object; } }
namespace rw { namespace proximity { class CollisionStrategy; } }
namespace rw { namespace proximity { class ProximityFilterStrategy; } }
namespace rw { namespace proximity { class ProximityModel; } }
namespace rw { namespace proximity { class ProximitySetup; } }
namespace rw { namespace proximity { class ProximitySetupRule; } }
namespace rwsim { namespace contacts { class ContactDetector; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Maintains a broad-phase filter where bodies can be easily added and removed.
 *
 * This is useful for TNTWorld and TNTIsland engines where bodies are added and removed
 * dynamically.
 *
 * Furthermore the class implements a function for detecting if there are penetrations so
 * large that contact detection is not possible. This can for instance be a problem if there
 * are penetrating contacts between trimesh geometries.
 */
class TNTBroadPhase {
public:
	//! @brief Definition of a constant frame pair.
	typedef std::pair<const rw::kinematics::Frame*, const rw::kinematics::Frame*> FramePairConst;
	//! @brief Definition of a list of constant frame pairs.
	typedef std::vector<FramePairConst> FramePairList;

	/**
	 * @brief Construct a new broad-phase checker.
	 * @param dwc [in] the DynamicWorkCell
	 */
	TNTBroadPhase(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	//! @brief Destructor.
	virtual ~TNTBroadPhase();

	/**
	 * @brief Add a new object to the broad-phase checker.
	 * @param object [in] the object to add.
	 * @param dynamic [in] true (default) if the object is a dynamic object, false otherwise.
	 */
	void addObject(rw::common::Ptr<rw::models::Object> object, bool dynamic = true);

	/**
	 * @brief Remove a object from the broad-phase checker.
	 * @param object [in] the object to remove.
	 */
	void removeObject(rw::common::Ptr<rw::models::Object> object);

	/**
	 * @brief Get the frame pairs that passes the broad-phase filtering.
	 * @param state [in] the state to check.
	 * @return a list of frame pairs.
	 */
	FramePairList broadPhase(const rw::kinematics::State& state);

	/**
	 * @brief Check if there are penetrations for detected contacts using the
	 * rwsim::contacts::ContactStrategyPQP contact strategy.
	 * @param detector [in] the contact detector used (should use the ContactStrategyPQP strategy).
	 * @param state [in] the state to check.
	 * @return true if there are penetrations, false otherwise.
	 */
	bool maxPenetrationExceeded(rw::common::Ptr<const rwsim::contacts::ContactDetector> detector, const rw::kinematics::State& state);

	/**
	 * @brief Get a pointer to underlying the broad-phase strategy used.
	 *
	 * This can for instance be used together with other collision/distance/contact detectors.
	 *
	 * @return a pointer to the rw::proximity::ProximityFilterStrategy
	 */
	rw::proximity::ProximityFilterStrategy* getProximityFilterStrategy() const;

private:
	static rw::proximity::ProximityFilterStrategy* getEmptyBPStrategy(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);
	static const rw::proximity::ProximitySetup* getDefaultProximitySetup(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	const rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
	const rw::proximity::ProximitySetup* const _defaultProximitySetup;

	rw::proximity::ProximityFilterStrategy* const _bpStrategy;
	rw::proximity::CollisionStrategy* const _collisionStrategy;

	std::set<const rw::kinematics::Frame*> _frames;
	std::set<const rw::kinematics::Frame*> _dynFrames;
	rw::kinematics::FrameMap<std::list<rw::proximity::ProximitySetupRule> >* _frameToBPRule;
	rw::kinematics::FrameMap<std::map<rw::common::Ptr<rw::geometry::Geometry>, rw::common::Ptr<rw::proximity::ProximityModel> > >* _frameGeoToPModel;

};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTBROADPHASE_HPP_ */
