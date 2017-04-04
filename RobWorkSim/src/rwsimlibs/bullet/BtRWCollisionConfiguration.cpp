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

#include <rw/common/macros.hpp>

#if BT_BULLET_VERSION > 281
#include "BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h"
#endif
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "LinearMath/btPoolAllocator.h"
#if BT_BULLET_VERSION < 282
#include "LinearMath/btStackAlloc.h"
#endif

using namespace rwsim::contacts;
using namespace rwsimlibs::bullet;

BtRWCollisionConfiguration::BtRWCollisionConfiguration(rw::common::Ptr<const ContactDetector> detector) {
	void* mem;

	int maxSize = 0;

#if BT_BULLET_VERSION > 281
	mem = btAlignedAlloc(sizeof(btCompoundCompoundCollisionAlgorithm::CreateFunc),16);
	m_compoundCompoundCreateFunc = new (mem)btCompoundCompoundCollisionAlgorithm::CreateFunc;
	maxSize = btMax(maxSize,static_cast<int>(sizeof(btCompoundCompoundCollisionAlgorithm)));
#else
	m_compoundCompoundCreateFunc = NULL;
#endif

	mem = btAlignedAlloc(sizeof(btCompoundCollisionAlgorithm::CreateFunc),16);
	m_compoundCreateFunc = new (mem)btCompoundCollisionAlgorithm::CreateFunc;

	mem = btAlignedAlloc(sizeof(btCompoundCollisionAlgorithm::SwappedCreateFunc),16);
	m_swappedCompoundCreateFunc = new (mem)btCompoundCollisionAlgorithm::SwappedCreateFunc;

	maxSize = btMax(maxSize,static_cast<int>(sizeof(btCompoundCollisionAlgorithm)));

	mem = btAlignedAlloc(sizeof(BtRWCollisionAlgorithm::CreateFunc),16);
	_func = new(mem) BtRWCollisionAlgorithm::CreateFunc(detector);
	maxSize = btMax(maxSize,static_cast<int>(sizeof(BtRWCollisionAlgorithm)));

	mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
	m_persistentManifoldPool = new (mem) btPoolAllocator(sizeof(btPersistentManifold),4096);

	mem = btAlignedAlloc(sizeof(btPoolAllocator),16);
	m_collisionAlgorithmPool = new(mem) btPoolAllocator(maxSize,4096);

#if BT_BULLET_VERSION < 282
	mem = btAlignedAlloc(sizeof(btStackAlloc),16);
	m_stackAlloc = new(mem)btStackAlloc(0);
#endif
}

BtRWCollisionConfiguration::~BtRWCollisionConfiguration() {
	m_collisionAlgorithmPool->~btPoolAllocator();
	btAlignedFree(m_collisionAlgorithmPool);

	m_persistentManifoldPool->~btPoolAllocator();
	btAlignedFree(m_persistentManifoldPool);

#if BT_BULLET_VERSION > 281
	m_compoundCompoundCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(m_compoundCompoundCreateFunc);
#endif

	m_compoundCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree( m_compoundCreateFunc);

	m_swappedCompoundCreateFunc->~btCollisionAlgorithmCreateFunc();
	btAlignedFree( m_swappedCompoundCreateFunc);

	_func->~btCollisionAlgorithmCreateFunc();
	btAlignedFree(_func);

#if BT_BULLET_VERSION < 282
	m_stackAlloc->destroy();
	m_stackAlloc->~btStackAlloc();
	btAlignedFree(m_stackAlloc);
#endif
}

btPoolAllocator* BtRWCollisionConfiguration::getPersistentManifoldPool() {
	return m_persistentManifoldPool;
}

btPoolAllocator* BtRWCollisionConfiguration::getCollisionAlgorithmPool() {
	return m_collisionAlgorithmPool;
}

#if BT_BULLET_VERSION < 282
btStackAlloc* BtRWCollisionConfiguration::getStackAllocator() {
	return m_stackAlloc;
}
#endif

btCollisionAlgorithmCreateFunc* BtRWCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0, int proxyType1) {
#if BT_BULLET_VERSION > 281
	if (btBroadphaseProxy::isCompound(proxyType0) && btBroadphaseProxy::isCompound(proxyType1)) {
		return m_compoundCompoundCreateFunc;
	}
#endif

	if (btBroadphaseProxy::isCompound(proxyType0)) {
		return m_compoundCreateFunc;
	} else if (btBroadphaseProxy::isCompound(proxyType1)) {
		return m_swappedCompoundCreateFunc;
	}

	return _func;
}

#if BT_BULLET_VERSION >= 286
btCollisionAlgorithmCreateFunc* BtRWCollisionConfiguration::getClosestPointsAlgorithmCreateFunc(int proxyType0, int proxyType1) {
	RW_THROW("Bullets getClosestPointsAlgorithmCreateFunc function was added in Bullet 2.86. It is not yet implemented.");
}
#endif