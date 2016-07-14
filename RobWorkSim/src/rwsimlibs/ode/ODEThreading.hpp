/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_ODE_ODETHREADING_HPP_
#define RWSIMLIBS_ODE_ODETHREADING_HPP_

/**
 * @file ODEThreading.hpp
 *
 * \copydoc rwsim::simulator::ODEThreading
 */

#include <rw/common/Ptr.hpp>

#include <ode/common.h>

namespace rwsim {
namespace simulator {
//! @addtogroup rwsim_simulator

//! @{
/**
 * @brief Utility functions related to the use of Open Dynamics Engine from multiple threads.
 *
 * For version 0.12 and earlier, using ODE from multiple threads might work (there is no guarantees).
 *
 * In version 0.13 a new structure for multi-threading was introduced. Open Dynamics Engine must be compiled with the
 * flags --enable-builtin-threading-impl and --enable-ou. This is required to support multi-threading in ODE 0.13 and newer.
 */
class ODEThreading {
public:
	virtual ~ODEThreading();

	/**
	 * @brief Make sure that threading is supported.
	 * @throws Exception if Open Dynamics Engine does not support threading.
	 * @see isSupported
	 */
	static void assertSupported();

	/**
	 * @brief Check if threading is supported.
	 *
	 * The function will return false for ODE 0.13 and newer if ODE is compiled without the --enable-builtin-threading-impl option.
	 *
	 * @return true if supported, false otherwise.
	 * @see assertSupported
	 */
	static bool isSupported();

	/**
	 * @brief If multiple threads try to execute dWorldStep or dWorldQuickStep simultaneously, they should call
	 * this before each step, followed by checkSecureStepEnd() after the step.
	 * This will give user-friendly errors if threading is not supported.
	 * @throws Exception if threading is not supported and this has been called more than once.
	 */
	static void checkSecureStepBegin();

	/**
	 * @brief Call this after all dWorldStep and dWorldQuickStep.
	 * @see checkSecureStepBegin
	 */
	static void checkSecureStepEnd();

	/**
	 * @brief Initialize threading structures (only used in ODE 0.13 and newer).
	 *
	 * Notice that this will create a worker thread for each world.
	 *
	 * @param world [in] the world id.
	 */
	static void initThreading(dWorldID world);

	/**
	 * @brief Destruct threading structures.
	 * @param world [in] the world id.
	 */
	static void destroyThreading(dWorldID world);

private:
	ODEThreading();

	struct ThreadImpl;
	static rw::common::Ptr<ThreadImpl> data();
};
//! @}
} /* namespace simulator */
} /* namespace rwsim */

#endif /* RWSIMLIBS_ODE_ODETHREADING_HPP_ */
