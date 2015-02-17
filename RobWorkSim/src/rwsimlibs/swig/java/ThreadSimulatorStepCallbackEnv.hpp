/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_SWIG_THREADSIMULATORSTEPCALLBACKENV_HPP_
#define RWSIMLIBS_SWIG_THREADSIMULATORSTEPCALLBACKENV_HPP_

/**
 * @file ThreadSimulatorStepCallbackEnv.hpp
 *
 * \copydoc rwsimlibs::swig::ThreadSimulatorStepCallbackEnv
 */

#include <rwsim/simulator/ThreadSimulator.hpp>

namespace rwsimlibs {
namespace swig {
//! @addtogroup rwsimlibs_swig

//! @{
/**
 * @brief An extension to the StepCallback function defined in ThreadSimulator that allows saving additional environment/user data.
 */
class ThreadSimulatorStepCallbackEnv: public rwsim::simulator::ThreadSimulator::StepCallback {
public:
	typedef void (*cThreadSimulatorStepCallback)(rwsim::simulator::ThreadSimulator*, rw::kinematics::State&, void*);

	ThreadSimulatorStepCallbackEnv(const ThreadSimulatorStepCallbackEnv &cb);
	ThreadSimulatorStepCallbackEnv(cThreadSimulatorStepCallback fct, void *userdata);
	virtual ~ThreadSimulatorStepCallbackEnv();
	void set(cThreadSimulatorStepCallback fct, void *userdata);
	void operator()(rwsim::simulator::ThreadSimulator* sim, rw::kinematics::State& state);
private:
	void callback(rwsim::simulator::ThreadSimulator* sim, rw::kinematics::State& state, void* data);

	cThreadSimulatorStepCallback _cb;
	void* _data;
};
//! @}
} /* namespace swig */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_SWIG_THREADSIMULATORSTEPCALLBACKENV_HPP_ */
