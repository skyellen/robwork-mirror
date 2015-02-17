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

#include "ThreadSimulatorStepCallbackEnv.hpp"

using namespace rw::kinematics;
using namespace rwsim::simulator;
using namespace rwsimlibs::swig;

ThreadSimulatorStepCallbackEnv::ThreadSimulatorStepCallbackEnv(const ThreadSimulatorStepCallbackEnv &cb):
	ThreadSimulator::StepCallback(boost::bind(&ThreadSimulatorStepCallbackEnv::callback, this, _1, _2, cb._data)),
	_cb(cb._cb),
	_data(cb._data)
{
}
ThreadSimulatorStepCallbackEnv::ThreadSimulatorStepCallbackEnv(cThreadSimulatorStepCallback fct, void *userdata):
	ThreadSimulator::StepCallback(boost::bind(&ThreadSimulatorStepCallbackEnv::callback, this, _1, _2, userdata)),
	_cb(fct),
	_data(userdata)
{
}
ThreadSimulatorStepCallbackEnv::~ThreadSimulatorStepCallbackEnv() {
}

void ThreadSimulatorStepCallbackEnv::set(cThreadSimulatorStepCallback fct, void *userdata) {
	ThreadSimulator::StepCallback::operator=(boost::bind(&ThreadSimulatorStepCallbackEnv::callback, this, _1, _2, userdata));
	_cb = fct;
	_data = userdata;
}

void ThreadSimulatorStepCallbackEnv::operator()(ThreadSimulator* sim, State& state) {
	_cb(sim, state, _data);
}

void ThreadSimulatorStepCallbackEnv::callback(ThreadSimulator* sim, State& state, void* data) {
	_cb(sim,state,data);
}
