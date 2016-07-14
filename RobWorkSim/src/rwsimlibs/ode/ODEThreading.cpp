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

#include "ODEThreading.hpp"

#include <rw/common/macros.hpp>

#ifdef ODE_WITH_THREADING_IMPL
#include <ode/odeinit.h>
#include <ode/objects.h>
#include <ode/threading_impl.h> // since ODE 0.13
#endif

#include <boost/thread/mutex.hpp>

using rw::common::ownedPtr;
using rwsim::simulator::ODEThreading;

#ifdef ODE_WITH_THREADING_IMPL
namespace {
boost::mutex counterMutex;
unsigned int counter = 0;
}

struct ODEThreading::ThreadImpl {
	std::map<dWorldID, dThreadingImplementationID> threading;
	std::map<dWorldID, dThreadingThreadPoolID> pool;
};

#else
struct ODEThreading::ThreadImpl {};
#endif

ODEThreading::ODEThreading() {
}

ODEThreading::~ODEThreading() {
}

void ODEThreading::assertSupported() {
#ifdef ODE_WITH_THREADING_IMPL
	if (!isSupported()) {
		RW_THROW("Open Dynamics Engine does not support threading - please make sure that ODE has been compiled with the --enable-builtin-threading-impl option.");
	}
#endif
}

bool ODEThreading::isSupported() {
#ifdef ODE_WITH_THREADING_IMPL
	static const int TLS = dCheckConfiguration("ODE_EXT_mt_collisions");
	static const int builtin_threading = dCheckConfiguration("ODE_THR_builtin_impl");
	static const bool supported = (TLS != 0) && (builtin_threading != 0);
	return supported;
#else
	return true;
#endif
}

void ODEThreading::checkSecureStepBegin() {
#ifdef ODE_WITH_THREADING_IMPL
	static bool supported = isSupported();
	boost::mutex::scoped_lock lock(counterMutex);
	counter++;
	if (!supported && counter > 1)
		RW_THROW("Open Dynamics Engine does not support multiple threads calling dWorldStep or dWorldQuickStep function simultaneously!");
#endif
}

void ODEThreading::checkSecureStepEnd() {
#ifdef ODE_WITH_THREADING_IMPL
	boost::mutex::scoped_lock lock(counterMutex);
	counter--;
#endif
}

void ODEThreading::initThreading(dWorldID world) {
#ifdef ODE_WITH_THREADING_IMPL
	static bool supported = isSupported();
	if (supported) {
		static const rw::common::Ptr<ThreadImpl> data = ODEThreading::data();
		data->threading[world] = dThreadingAllocateMultiThreadedImplementation();
		data->pool[world] = dThreadingAllocateThreadPool(1, 0, dAllocateFlagBasicData, NULL);
		dThreadingThreadPoolServeMultiThreadedImplementation(data->pool[world], data->threading[world]);
		dWorldSetStepThreadingImplementation(world, dThreadingImplementationGetFunctions(data->threading[world]), data->threading[world]);
	}
#endif
}


void ODEThreading::destroyThreading(dWorldID world) {
#ifdef ODE_WITH_THREADING_IMPL
	static bool supported = isSupported();
	if (supported) {
		static const rw::common::Ptr<ThreadImpl> data = ODEThreading::data();
		dThreadingImplementationShutdownProcessing(data->threading[world]);
		dThreadingFreeThreadPool(data->pool[world]);
		dWorldSetStepThreadingImplementation(world, NULL, NULL);
		dThreadingFreeImplementation(data->threading[world]);
	}
#endif
}

rw::common::Ptr<ODEThreading::ThreadImpl> ODEThreading::data() {
	static const rw::common::Ptr<ThreadImpl> data = ownedPtr(new ThreadImpl());
	return data;
}
