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

#ifndef RWSIMLIBS_RWPE_RWPELOGUTIL_HPP_
#define RWSIMLIBS_RWPE_RWPELOGUTIL_HPP_

//! @brief Macro for easy logging.
#define RWPE_ENGINE_LOG(object,ostreamExpression)    \
if (!(object == NULL)) {                           \
	std::stringstream stream;                      \
	stream << ostreamExpression;                   \
	object->log(stream.str(), __FILE__, __LINE__); \
}

//! @brief Macro that works as two arguments for file and line.
#define RWPE_LOCATION __FILE__,__LINE__

/**
 * @file RWPELogUtil.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPELogUtil
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Wrench6D.hpp>
#include <rw/proximity/CollisionStrategy.hpp>

namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace contacts { class Contact; } }
namespace rwsim { namespace contacts { class Constraints; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }
namespace rwsim { namespace log { class LogMessage; } }

namespace rwsimlibs {
namespace rwpe {

class RWPEBodyConstraintGraph;
class RWPEIslandState;
class RWPEContact;
class RWPEConstraint;

//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class RWPELogUtil {
public:
	RWPELogUtil();
	virtual ~RWPELogUtil();

	rwsim::log::SimulatorLogScope* makeScope(const std::string& name, const char* file = "", int line = -1);

	void setSimulatorLog(rw::common::Ptr<rwsim::log::SimulatorLogScope> log);

	bool doLog() const;

	virtual void beginStep(double time, const char* file = "", int line = -1);

	virtual void endStep(double time, int line = -1);

	virtual void beginSection(const std::string& name, const char* file = "", int line = -1);

	virtual void endSection(int line = -1);

	virtual void addCollisionResults(const std::string& description, const std::vector<rw::proximity::CollisionStrategy::Result>& results, const char* file = "", int line = -1);

	virtual void addContacts(const std::string& description, const std::vector<rwsim::contacts::Contact>& contacts, const char* file = "", int line = -1);
	virtual void addContacts(const std::string& description, const std::vector<const RWPEContact*>& contacts, const char* file = "", int line = -1);

	virtual void addConstraints(const std::string& description, const std::vector<const RWPEConstraint*>& constraints, const char* file = "", int line = -1);

	virtual void addPositions(const std::string& description, const std::map<std::string,rw::math::Transform3D<> >& positions, const char* file = "", int line = -1);
	virtual void addPositions(const std::string& description, const RWPEBodyConstraintGraph* bc, const rw::kinematics::State& state, const char* file = "", int line = -1);
	virtual void addPositions(const std::string& description, const RWPEBodyConstraintGraph* bc, const RWPEIslandState& state, const char* file = "", int line = -1);

	virtual void addVelocities(const std::string& description, const std::map<std::string,rw::math::VelocityScrew6D<> >& velocities, const char* file = "", int line = -1) ;
	virtual void addVelocities(const std::string& description, const RWPEBodyConstraintGraph* bc, const rw::kinematics::State& rwstate, const RWPEIslandState& islandState, const char* file = "", int line = -1) ;

	virtual void addWrenches(const std::string& description, const std::map<std::string,rw::math::Wrench6D<> >& ft, const char* file = "", int line = -1) ;

	virtual void addContactVelocities(const std::string& description, const std::vector<const RWPEContact*>& contacts, const rw::kinematics::State& rwstate, const RWPEIslandState& islandState, const char* file = "", int line = -1) ;
	typedef enum WrenchType {
		TOTAL,
		APPLIED,
		CONSTRAINT
	} WrenchType;
	virtual void addContactWrenches(const std::string& description, const std::vector<const RWPEContact*>& contacts, const RWPEIslandState& islandState, WrenchType type = TOTAL, const char* file = "", int line = -1) ;

	virtual void addContactTracking(const std::string& description, const RWPEIslandState& islandState0, const RWPEIslandState& islandStateH, const char* file = "", int line = -1) ;

	virtual void addValues(const std::string& description, const std::vector<double>& values, const std::vector<std::string>& labels, const char* file = "", int line = -1);

	virtual std::ostream& log(const std::string& description, const char* file = "", int line = -1);

	virtual std::ostream& log(const char* file = "", int line = -1);

	virtual RWPELogUtil* parallel(const std::string& description, const char* file = "", int line = -1);

private:
	rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;
	rwsim::log::SimulatorLogScope* _scope;
	std::stringstream _dummyStream;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPELOGUTIL_HPP_ */
