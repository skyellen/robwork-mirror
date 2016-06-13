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

#include "BtRWCollisionAlgorithm.hpp"
#include "BtBody.hpp"
#include "BtUtil.hpp"

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include <rwsim/contacts/ContactDetector.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::math;
using namespace rwsim::contacts;
using namespace rwsimlibs::bullet;

BtRWCollisionAlgorithm::BtRWCollisionAlgorithm(
		rw::common::Ptr<const ContactDetector> detector,
		const btCollisionAlgorithmConstructionInfo& ci,
		const btCollisionObjectWrapper* col0Wrap,
		const btCollisionObjectWrapper* col1Wrap):
	btCollisionAlgorithm(ci),
	_detector(detector),
	_stratData(new ContactStrategyData())
{
	m_manifoldPtr = m_dispatcher->getNewManifold(col0Wrap->getCollisionObject(),col1Wrap->getCollisionObject());
	// Contact breaking threshold determines when a contact will be bouncing or sticking!
	// - low threshold: lot of bouncing
	// - high threshold: less bouncing
	//m_manifoldPtr->setContactBreakingThreshold(0.01);
}

BtRWCollisionAlgorithm::~BtRWCollisionAlgorithm() {
	m_dispatcher->releaseManifold(m_manifoldPtr);
	delete _stratData;
}

void BtRWCollisionAlgorithm::processCollision(
		const btCollisionObjectWrapper* body0Wrap,
		const btCollisionObjectWrapper* body1Wrap,
		const btDispatcherInfo& dispatchInfo,
		btManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
	    RW_THROW("BtRWCollisionAlgorithm failed in processCollision: internal m_manifoldPtr was null!");
	if (!resultOut)
		RW_THROW("BtRWCollisionAlgorithm could not process collision as btManifoldResult pointer was null.");

	const BtBody::BodyMetaData* const dataA = static_cast<BtBody::BodyMetaData*>(body0Wrap->getCollisionObject()->getUserPointer());
	const BtBody::BodyMetaData* const dataB = static_cast<BtBody::BodyMetaData*>(body1Wrap->getCollisionObject()->getUserPointer());
	const BtBody::GeometryMetaData* const geoA = static_cast<const BtBody::GeometryMetaData*>(body0Wrap->getCollisionShape()->getUserPointer());
	const BtBody::GeometryMetaData* const geoB = static_cast<const BtBody::GeometryMetaData*>(body1Wrap->getCollisionShape()->getUserPointer());
	RW_ASSERT(dataA);
	RW_ASSERT(dataB);
	RW_ASSERT(geoA);
	RW_ASSERT(geoB);

	const Transform3D<> T0 = BtUtil::toTransform3D(body0Wrap->getWorldTransform().getOrigin(),body0Wrap->getWorldTransform().getRotation());
	const Transform3D<> T1 = BtUtil::toTransform3D(body1Wrap->getWorldTransform().getOrigin(),body1Wrap->getWorldTransform().getRotation());

	const Transform3D<> comTframeA = inverse(geoA->geometry->getTransform());
	const Transform3D<> comTframeB = inverse(geoB->geometry->getTransform());

	const ContactDetector::StrategyTable strategies = _detector->getContactStrategies(dataA->frame->getName(),geoA->geometry->getGeometryData(),dataB->frame->getName(),geoB->geometry->getGeometryData());
	if (strategies.empty())
		return;

	const ContactDetector::StrategyTableRow& match = strategies.front();

	resultOut->setPersistentManifold(m_manifoldPtr);

	const std::map<std::string, ContactModel::Ptr>& mapA = match.models[*dataA->frame];
	const std::map<std::string, ContactModel::Ptr>& mapB = match.models[*dataB->frame];
	const std::map<std::string, ContactModel::Ptr>::const_iterator modelA = mapA.find(geoA->geometry->getId());
	const std::map<std::string, ContactModel::Ptr>::const_iterator modelB = mapB.find(geoB->geometry->getId());
	RW_ASSERT(modelA != mapA.end());
	RW_ASSERT(modelB != mapB.end());
	const std::vector<Contact> contacts = match.strategy->findContacts(modelA->second,T0*comTframeA,modelB->second,T1*comTframeB,*_stratData);

	BOOST_FOREACH(const Contact& c, contacts) {
		const btScalar dist = -c.getDepth();
		btVector3 pos1;
		btVector3 n;
		if (c.getFrameA() == dataA->frame) {
			pos1 = BtUtil::makeBtVector(c.getPointB());
			n = BtUtil::makeBtVector(-c.getNormal());
		} else {
			pos1 = BtUtil::makeBtVector(c.getPointA());
			n = BtUtil::makeBtVector(c.getNormal());
		}
		resultOut->addContactPoint(n,pos1,dist);
	}

	resultOut->refreshContactPoints();
}

btScalar BtRWCollisionAlgorithm::calculateTimeOfImpact(
		btCollisionObject* body0,
		btCollisionObject* body1,
		const btDispatcherInfo& dispatchInfo,
		btManifoldResult* resultOut) {
	RW_THROW("BtRWCollisionAlgorithm (calculateTimeOfImpact): continuous contact detection is not supported!");
}

void BtRWCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
	if (!m_manifoldPtr)
	    RW_THROW("BtRWCollisionAlgorithm failed in processCollision: internal m_manifoldPtr was null!");
	manifoldArray.push_back(m_manifoldPtr);
}

BtRWCollisionAlgorithm::CreateFunc::CreateFunc(rw::common::Ptr<const ContactDetector> detector):
	_detector(detector)
{
}

BtRWCollisionAlgorithm::CreateFunc::~CreateFunc() {
}

btCollisionAlgorithm* BtRWCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap) {
	void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(BtRWCollisionAlgorithm));
	return new(mem) BtRWCollisionAlgorithm(_detector,ci,body0Wrap,body1Wrap);
}
