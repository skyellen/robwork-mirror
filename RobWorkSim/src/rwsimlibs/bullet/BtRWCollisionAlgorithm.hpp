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

#ifndef RWSIMLIBS_BULLET_BTRWCOLLISIONALGORITHM_HPP_
#define RWSIMLIBS_BULLET_BTRWCOLLISIONALGORITHM_HPP_

/**
 * @file BtRWCollisionAlgorithm.hpp
 *
 * \copydoc rwsimlibs::bullet::BtRWCollisionAlgorithm
 */

#include <rw/common/Ptr.hpp>

#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btCollisionCreateFunc.h>

namespace rwsim { namespace contacts { class ContactDetector; } }
namespace rwsim { namespace contacts { class ContactStrategyData; } }

namespace rwsimlibs {
namespace bullet {
//! @addtogroup rwsimlibs_bullet

//! @{
/**
 * @brief The RobWork implementation of a Bullet collision algorithm, using the standard ContactDetector.
 */
class BtRWCollisionAlgorithm: public btCollisionAlgorithm {
public:
	/**
	 * @brief Construct new collision algorithm for Bullet that wraps a RobWork ContactDetector.
	 * @param detector [in] the contact detector to use.
	 * @param ci [in] the construction info.
	 * @param col0Wrap [in] information about the first bullet body.
	 * @param col1Wrap [in] information about the second bullet body.
	 */
	BtRWCollisionAlgorithm(rw::common::Ptr<const rwsim::contacts::ContactDetector> detector,
			const btCollisionAlgorithmConstructionInfo& ci,
			const btCollisionObjectWrapper* col0Wrap,
			const btCollisionObjectWrapper* col1Wrap);

	//! @brief Destructor.
	virtual ~BtRWCollisionAlgorithm();

	/**
	 * @brief Find the contacts.
	 * @param body0Wrap [in] information about the first bullet body.
	 * @param body1Wrap [in] information about the second bullet body.
	 * @param dispatchInfo [in] info about the dispatcher.
	 * @param resultOut [out] the resulting contacts.
	 * @throws Exception if resultOut is null or no manifold could be retrieved from dispatcher.
	 */
	virtual void processCollision(
			const btCollisionObjectWrapper* body0Wrap,
			const btCollisionObjectWrapper* body1Wrap,
			const btDispatcherInfo& dispatchInfo,
			btManifoldResult* resultOut);

	/**
	 * @brief Calculate the time of impact (not supported).
	 * @param body0 [in] information about the first bullet body.
	 * @param body1 [in] information about the second bullet body.
	 * @param dispatchInfo [in] info about the dispatcher.
	 * @param resultOut [out] the resulting contacts.
	 * @return the time of impact.
	 * @throws Exception always (not supported).
	 */
	virtual btScalar calculateTimeOfImpact(
			btCollisionObject* body0,
			btCollisionObject* body1,
			const btDispatcherInfo& dispatchInfo,
			btManifoldResult* resultOut);

	/**
	 * @brief Get all contact manifolds.
	 * @note The contact manifold will be appended to the given array.
	 * @param manifoldArray [out] get all manifolds.
	 * @throws Exception if no manifold could be retrieved from dispatcher.
	 */
	virtual	void getAllContactManifolds(btManifoldArray& manifoldArray);

	//! @brief Function for creating a BtRWCollisionAlgorithm
	struct CreateFunc: public btCollisionAlgorithmCreateFunc {
		/**
		 * @brief New factory.
		 * @param detector [in] the detector to use.
		 */
		CreateFunc(rw::common::Ptr<const rwsim::contacts::ContactDetector> detector);

		//! @brief Destructor.
		virtual ~CreateFunc();

		/**
		 * @brief Construct a new algorithm for a object pair.
		 * @param ci [in] construction info for a btCollisionAlgorithm.
		 * @param body0Wrap [in] information about the first bullet body.
	 	 * @param body1Wrap [in] information about the second bullet body.
		 * @return a pointer to the new algorithm.
		 */
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap);

	private:
		rw::common::Ptr<const rwsim::contacts::ContactDetector> _detector;
	};

private:
	rw::common::Ptr<const rwsim::contacts::ContactDetector> _detector;
	btPersistentManifold* m_manifoldPtr;
	rwsim::contacts::ContactStrategyData* _stratData;
};
//! @}
} /* namespace bullet */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_BULLET_BTRWCOLLISIONALGORITHM_HPP_ */
