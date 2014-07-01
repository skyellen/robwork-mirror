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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTBODY_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTBODY_HPP_

/**
 * @file TNTBody.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTBody
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Transform3D.hpp>

// Forward declarations
namespace rw { namespace kinematics { class State; } }
namespace rwsim { namespace dynamics { class Body; } }

namespace rwsimlibs {
namespace tntphysics {

// Forward declarations
class TNTIslandState;

//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief The TNTBody is a wrapper for a rwsim::dynamics::Body, which allows storing data
 * in a TNTIslandState instead of rw::kinematics::State.
 *
 * Functions are provided for converting back and forth between the different state structures.
 *
 * The TNTIslandState structure allows bodies and associated integrators to use own custom
 * position and velocity representations if desired.
 *
 * Under normal circumstances it is desirable to store the body position in world coordinates.
 * In the rw::kinematics::State structure these are stored as relative positions instead.
 * The TNTBody type allows using the TNTIslandState structure such that world coordinates can
 * be used internally by the physics engine.
 */
class TNTBody {
public:
	//! @brief Destructor.
	virtual ~TNTBody();

	/**
	 * @brief Get the wrapped rwsim::dynamics::Body
	 * @return a smart pointer to the wrapped body.
	 */
	rw::common::Ptr<rwsim::dynamics::Body> get() const;

	/**
	 * @brief The configuration of a body. Bodies can inherit and extend this to use
	 * other representations internally.
	 * The basic Configuration of a body is the position as a Transform3D.
	 * If other representations are used, they must be able to convert back and forth
	 * between the Transform3D.
	 */
	class Configuration {
	public:
		/**
		 * @brief Constructor.
		 * @param worldTcom [in] (optional) the center of mass of the body in world coordinates.
		 */
		Configuration(rw::math::Transform3D<> worldTcom = rw::math::Transform3D<>::identity()) { _worldTcom = worldTcom; };

		//! @brief Destructor.
		virtual ~Configuration() {};

		/**
		 * @brief Get the current position of the center of mass of the body.
		 * @return position of center of mass relative to world frame.
		 */
		virtual rw::math::Transform3D<> getWorldTcom() const { return _worldTcom; };

		/**
		 * @brief Set the current position of the center of mass of the body.
		 * @param worldTcom [in] the center of mass relative to the world frame.
		 */
		virtual void setWorldTcom(const rw::math::Transform3D<>& worldTcom) { _worldTcom = worldTcom; };

		/**
		 * @brief Clone the configuration.
		 * @return a new configuration that is a copy of the original.
		 */
		virtual Configuration* clone() const { return new Configuration(_worldTcom); };

	private:
		rw::math::Transform3D<> _worldTcom;
	};

	/**
	 * @brief Construct a new Configuration based on the given state.
	 *
	 * Subtypes of TNTBody can re-implement this function to use custom
	 * data.
	 *
	 * @param rwstate [in] the state to construct Configuration based on.
	 * @return pointer to a Configuration - owned by the caller.
	 */
	virtual Configuration* makeConfiguration(const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Get the current transformation of the body center of mass in world coordinates.
	 * @param tntstate [in] the state to find transformation for.
	 * @return the transformation.
	 */
	virtual rw::math::Transform3D<> getWorldTcom(const TNTIslandState &tntstate) const;

	/**
	 * @brief Set a new transformation of the body center of mass.
	 * @param wTcom [in] the new body transformation in world coordinates.
	 * @param tntstate [in/out] the state to update with the new position.
	 */
	virtual void setWorldTcom(const rw::math::Transform3D<> &wTcom, TNTIslandState &tntstate) const;

	/**
	 * @brief Get the velocity of the body center of mass.
	 *
	 * The velocity can be stored in both the TNTIslandState and the rw::kinematics::State structures.
	 * In some cases the velocity might be given directly in the rw::kinematics::State structure.
	 * If the simulator should be able to modify the velocity, it should however be saved in the TNTIslandState
	 * structure.
	 *
	 * @param rwstate [in] the current state.
	 * @param tntstate [in] the current TNTIslandState.
	 * @return velocity in world coordinates.
	 */
	virtual rw::math::VelocityScrew6D<> getVelocityW(const rw::kinematics::State &rwstate, const TNTIslandState &tntstate) const = 0;

	/**
	 * @brief Reload the TNTBody position and velocity to that given be the rw::kinematics::State.
	 * @param tntstate [in/out] the TNTIslandState to store the changed position and velocity to.
	 * @param rwstate [in] the rw::kinematics::State to read data from.
	 */
	virtual void reset(TNTIslandState &tntstate, const rw::kinematics::State &rwstate) const;

	/**
	 * @brief Update the rw::kinematics::State with the data in TNTIslandState.
	 * @param rwstate [in/out] the state to update.
	 * @param tntstate [in] the TNTIslandState to read data from.
	 */
	virtual void updateRW(rw::kinematics::State &rwstate, const TNTIslandState &tntstate) const = 0;

	/**
	 * @brief Construct a new Configuration based on the given state.
	 * @param body [in] the rwsim::dynamics::Body to construct Configuration for.
	 * @param rwstate [in] the state to construct Configuration based on.
	 * @return pointer to a Configuration - owned by the caller.
	 */
	static Configuration* getDefaultConfiguration(rw::common::Ptr<const rwsim::dynamics::Body> body, const rw::kinematics::State &rwstate);

protected:
	/**
	 * @brief Construct a new body.
	 * @param body [in] a pointer to the underlying rwsim::dynamics::Body.
	 */
	TNTBody(rw::common::Ptr<rwsim::dynamics::Body> body);

private:
	rw::common::Ptr<rwsim::dynamics::Body> _body;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_ */
