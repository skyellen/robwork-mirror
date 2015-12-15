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

#ifndef RWSIMLIBS_RWPE_RWPEBODYFIXED_HPP_
#define RWSIMLIBS_RWPE_RWPEBODYFIXED_HPP_

/**
 * @file RWPEBodyFixed.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEBodyFixed
 */

#include <rw/common/Ptr.hpp>
#include "RWPEBody.hpp"

// Forward declarations
namespace rwsim { namespace dynamics { class FixedBody; } }

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief The RWPEBodyFixed is a wrapper for a rwsim::dynamics::FixedBody and is used for bodies
 * that do not move.
 */
class RWPEBodyFixed: public RWPEBody {
public:
	/**
	 * @brief Construct a new fixed body.
	 * @param body [in] a pointer to the underlying rwsim::dynamics::FixedBody.
	 */
	RWPEBodyFixed(rw::common::Ptr<rwsim::dynamics::FixedBody> body);

	//! @brief Destructor.
	virtual ~RWPEBodyFixed();

	/**
	 * @brief Get the wrapped rwsim::dynamics::FixedBody
	 * @return the native object.
	 */
	rw::common::Ptr<rwsim::dynamics::FixedBody> getFixedBody() const;

	//! @copydoc RWPEBody::updateRW
	virtual void updateRW(rw::kinematics::State &rwstate, const RWPEIslandState &islandState) const;

	//! @copydoc RWPEBody::getVelocityW
	virtual rw::math::VelocityScrew6D<> getVelocityW(const rw::kinematics::State &rwstate, const RWPEIslandState &islandState) const;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEBODYFIXED_HPP_ */
