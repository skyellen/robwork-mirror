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

#include "BtRWCollisionConfiguration.hpp"
#include "BtRWCollisionAlgorithm.hpp"

#include "bullet/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h"
#include "bullet/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "bullet/LinearMath/btPoolAllocator.h"

using namespace rwsim::contacts;
using namespace rwsimlibs::bullet;

BtRWCollisionConfiguration::BtRWCollisionConfiguration(rw::common::Ptr<const ContactDetector> detector) {
	void* mem;

	mem = btAlignedAlloc(sizeof(btCompoundCompoundCollisionAlgorithm::CreateFunc),16);
	m_compoundCompoundCreateFunc = new (mem)btCompoundCompoundCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(BtRWCollisionAlgorithm::CreateFunc),16);
	_func = new(mem) BtRWCollisionAlgorithm::CreateFunc(detector);

	mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
	m_persistentManifoldPool = new (mem) btPoolAllocator(sizeof(btPersistentManifold),4096);

	mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
	const int size1 = sizeof(btCompoundCompoundCollisionAlgorithm);
	const int size2 = sizeof(BtRWCollisionAlgorithm);
	m_collisionAlgorithmPool = new(mem) btPoolAllocator(btMax(size1,size2),4096);
}

BtRWCollisionConfiguration::~BtRWCollisionConfiguration() {
	m_collisionAlgorithmPool->~btPoolAllocator();
	btAlignedFree(m_collisionAlgorithmPool);

	m_persistentManifoldPool->~btPoolAllocator();
	btAlignedFree(m_persistentManifoldPool);

	m_compoundCompoundCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_compoundCompoundCreateFunc);

	_func->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(_func);
}

btPoolAllocator* BtRWCollisionConfiguration::getPersistentManifoldPool() {
	return m_persistentManifoldPool;
}

btPoolAllocator* BtRWCollisionConfiguration::getCollisionAlgorithmPool() {
	return m_collisionAlgorithmPool;
}

btCollisionAlgorithmCreateFunc* BtRWCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1) {
	if (btBroadphaseProxy::isCompound(proxyType0) && btBroadphaseProxy::isCompound(proxyType1)) {
		return m_compoundCompoundCreateFunc;
	} else {
		return _func;
	}
}
