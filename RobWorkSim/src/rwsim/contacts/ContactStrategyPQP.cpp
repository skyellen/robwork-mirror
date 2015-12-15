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

#include "ContactStrategyPQP.hpp"
#include "ContactStrategyTracking.hpp"

#include <rw/proximity/ProximityCache.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/dynamics/OBRManifold.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwlibs::proximitystrategies;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;

struct ContactStrategyPQP::Model {
	std::string geoId;
	rw::geometry::TriMesh::Ptr mesh;
	rw::math::Transform3D<> transform;
	const rw::kinematics::Frame* frame;
	rw::proximity::ProximityModel::Ptr pmodel;
};

class ContactStrategyPQP::TriMeshModel: public ContactModel {
public:
	typedef rw::common::Ptr<TriMeshModel> Ptr;
	TriMeshModel(ContactStrategy *owner): ContactModel(owner) {}
	~TriMeshModel() {
		if (models.size() > 0) {
			RW_THROW("Please use destroyModel on ContactStrategyPQP to deallocate internal caching before deleting the ProximityModel.");
		}
	}
	virtual std::string getName() const { return "TriMeshModel"; }
	std::vector<Model> models;
};

class ContactStrategyPQP::PQPData: public ContactStrategyData::SpecificData {
public:
	PQPData() {
		data.setCollisionQueryType(CollisionStrategy::AllContacts);
	}
	virtual ~PQPData() {}
	virtual SpecificData* copy() const {
		PQPData* const cp = new PQPData();
		cp->data = data;
		return cp;
	}

public:
	ProximityStrategyData data;
};

class ContactStrategyPQP::PQPTracking: public ContactStrategyTracking::StrategyData {
public:
	PQPTracking() {}
	virtual ~PQPTracking() {}
	virtual const ContactStrategyTracking::UserData::Ptr getUserData(std::size_t index) const {
		RW_ASSERT(index < info.size());
		return info[index].userData;
	}
	virtual void setUserData(std::size_t index, const ContactStrategyTracking::UserData::Ptr data) {
		RW_ASSERT(index < info.size());
		info[index].userData = data;
	}
	virtual void remove(std::size_t index) {
		RW_ASSERT(index < info.size());
		std::vector<TrackInfo>::iterator it = info.begin()+index;
		const std::size_t total = it->total;
		const std::size_t id = it->id;
		RW_ASSERT(id < total);
		RW_ASSERT(total <= info.size());
		it = info.erase(it);
		it = it - id;
		for (std::size_t i = 0; i < total-1; i++) {
			it->total--;
			it->id = i;
			it++;
		}
	}
	virtual StrategyData* copy() const {
		PQPTracking* tracking = new PQPTracking();
		tracking->aTb = aTb;
		tracking->info = info;
		return tracking;
	}

	virtual std::size_t getSize() const {
		return info.size();
	}

	struct TrackInfo {
		std::size_t modelIDa;
		std::size_t modelIDb;
		ContactStrategyTracking::UserData::Ptr userData;
		rw::math::Vector3D<> posA;
		rw::math::Vector3D<> posB;
		std::size_t id;
		std::size_t total;
	};

	bool find(std::size_t a, std::size_t b, std::vector<TrackInfo>& res) const {
		res.clear();
		for (std::size_t i = 0; i < info.size(); i++) {
			if (info[i].modelIDa == a && info[i].modelIDb == b) {
				res.push_back(info[i]);
			}
		}
		return res.size() > 0;
	}

public:
	rw::math::Transform3D<> aTb;
	std::vector<TrackInfo> info;
};

ContactStrategyPQP::ContactStrategyPQP():
	_matchAll(true),
	_narrowStrategy(new ProximityStrategyPQP()),
	_filtering(MANIFOLD)
{
}

ContactStrategyPQP::~ContactStrategyPQP() {
	delete _narrowStrategy;
}

bool ContactStrategyPQP::match(rw::common::Ptr<const GeometryData> geoA, rw::common::Ptr<const GeometryData> geoB) {
	if (_matchAll)
		return true;
	else {
		bool gA = false;
		bool gB = false;
		if (geoA->getType() == GeometryData::LineMesh ||
				geoA->getType() == GeometryData::PlainTriMesh ||
				geoA->getType() == GeometryData::IdxTriMesh)
			gA = true;
		if (geoB->getType() == GeometryData::LineMesh ||
				geoB->getType() == GeometryData::PlainTriMesh ||
				geoB->getType() == GeometryData::IdxTriMesh)
			gB = true;
		if (gA && gB)
			return true;
	}
	return false;
}

void ContactStrategyPQP::findContact(std::vector<Contact> &contacts,
			const Model& a,	const Transform3D<>& wTa,
			const Model& b,	const Transform3D<>& wTb,
			PQPData* data,
			bool distCheck) const
{
	double threshold;

	// If distance check should be used, the ordinary distance threshold is used.
	// When known contacts should be updated, there should be no threshold as
	// the contacts can then be in any distance.
	// (for now the "unlimited threshold" is achieved by just making the threshold x times bigger)
	// Note that bigger threshold requires more computation time, so this is a trade-off.
	if (distCheck)
		threshold = getThreshold();
	else
		threshold = 3*getThreshold();

	MultiDistanceResult *res;

	res = &_narrowStrategy->distances(a.pmodel, wTa, b.pmodel, wTb, threshold, data->data);

	for(size_t i=0;i<res->distances.size();i++){
		Contact c;
		Vector3D<> p1 = wTa*res->p1s[i];
		Vector3D<> p2 = wTa*res->p2s[i];
		c.setPointA(p1);
		c.setPointB(p2);

		c.setFrameA(a.frame);
		c.setFrameB(b.frame);

		if(res->distances[i]<0.00000001){
			std::pair< Vector3D<>, Vector3D<> > normals = _narrowStrategy->getSurfaceNormals(*res, (int)i);
			// the second is described in b's refframe so convert both to world and combine them
			Vector3D<> a_normal = wTa.R() * normals.first;
			Vector3D<> b_normal = wTb.R() * normals.second;

			c.setNormal(-normalize( a_normal - b_normal ));
		} else {
			c.setNormal(normalize(p2-p1));
		}
		c.setDepth(-res->distances[i]);
		c.setTransform(inverse(wTa)*wTb);
		contacts.push_back(c);
	}

	res->clear();
}

std::vector<Contact> ContactStrategyPQP::findContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> contacts;
	const TriMeshModel::Ptr mA = a.cast<TriMeshModel>();
	const TriMeshModel::Ptr mB = b.cast<TriMeshModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!data.isInitialized())
		data.setSpecificData(new PQPData());
	PQPData* const pqpData = dynamic_cast<PQPData*>(data.getSpecificData());
	RW_ASSERT(pqpData);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new PQPTracking());
	PQPTracking* const pqpTracking = dynamic_cast<PQPTracking*>(tracking.getStrategyData());
	RW_ASSERT(pqpTracking);

	std::vector<PQPTracking::TrackInfo> newInfo;

	const Transform3D<> aTb = inverse(wTa)*wTb;
	const EAA<> dif(inverse(pqpTracking->aTb.R())*aTb.R());
	const double absoluteThreshold = getUpdateThresholdAbsolute();
	const double linThreshold = getUpdateThresholdLinear()*(pqpTracking->aTb.P()-aTb.P()).norm2();
	const double angThreshold = getUpdateThresholdAngular()*dif.angle();
	const double threshold = absoluteThreshold+linThreshold+angThreshold;

	for (std::size_t i = 0; i < mA->models.size(); i++) {
		for (std::size_t j = 0; j < mB->models.size(); j++) {
			std::vector<PQPTracking::TrackInfo> currentInfos;
			const bool oldContact = pqpTracking->find(i,j,currentInfos);
			const Model modelA = mA->models[i];
			const Model modelB = mB->models[j];

			std::vector<Contact> tmpContacts;
			findContact(tmpContacts, modelA, wTa, modelB, wTb, pqpData, !oldContact);
			BOOST_FOREACH(Contact &c, tmpContacts) {
				c.setModelA(mA);
				c.setModelB(mB);
			}

			if (_filtering == MANIFOLD) {
				tmpContacts = manifoldFilter(tmpContacts);
			}

			BOOST_FOREACH(Contact &c, tmpContacts) {
				c.setDepth(getThreshold()+c.getDepth());
			}

			std::size_t curId = 0;
			if (oldContact) {
				BOOST_FOREACH(const PQPTracking::TrackInfo& info, currentInfos) {
					const Vector3D<> posA = info.posA;
					const Vector3D<> posB = info.posB;
					std::vector<std::size_t> candidates;
					for (std::size_t k = 0; k < tmpContacts.size(); k++) {
						const Contact& c = tmpContacts[k];
						const Vector3D<> newPosA = inverse(wTa)*c.getPointA();
						const Vector3D<> newPosB = inverse(wTb)*c.getPointB();
						if ((newPosA-posA).norm2() <= threshold && (newPosB-posB).norm2() <= threshold) {
							candidates.push_back(k);
						}
					}
					if (candidates.size() > 0) {
						double depth = tmpContacts[candidates[0]].getDepth();
						std::size_t deepest = candidates[0];
						for (std::size_t k = 1; k < candidates.size(); k++) {
							double tmpDepth = tmpContacts[candidates[k]].getDepth();
							if (tmpDepth > depth) {
								depth = tmpDepth;
								deepest = candidates[k];
							}
						}
						contacts.push_back(tmpContacts[deepest]);
						newInfo.push_back(info);
						newInfo.back().posA = inverse(wTa)*tmpContacts[deepest].getPointA();
						newInfo.back().posB = inverse(wTb)*tmpContacts[deepest].getPointB();
						newInfo.back().id = curId;
						curId++;
						tmpContacts.erase(tmpContacts.begin()+deepest);
					}
				}
			}
			// Add remaining contacts
			PQPTracking::TrackInfo info;
			info.modelIDa = i;
			info.modelIDb = j;
			info.userData = NULL;
			BOOST_FOREACH(const Contact&c, tmpContacts) {
				if (c.getDepth() > 0) {
					info.id = curId;
					info.posA = inverse(wTa)*c.getPointA();
					info.posB = inverse(wTb)*c.getPointB();
					newInfo.push_back(info);
					curId++;
					contacts.push_back(c);
				}
			}
			tmpContacts.clear();
			BOOST_FOREACH(PQPTracking::TrackInfo& info, newInfo) {
				info.total = curId;
			}
		}
	}
	pqpTracking->aTb = inverse(wTa)*wTb;
	pqpTracking->info = newInfo;
	return contacts;
}

std::vector<Contact> ContactStrategyPQP::updateContacts(
		ProximityModel::Ptr a, const Transform3D<>& wTa,
		ProximityModel::Ptr b, const Transform3D<>& wTb,
		ContactStrategyData& data,
		ContactStrategyTracking& tracking,
		rwsim::log::SimulatorLogScope* log) const
{
	std::vector<Contact> contacts;
	const TriMeshModel::Ptr mA = a.cast<TriMeshModel>();
	const TriMeshModel::Ptr mB = b.cast<TriMeshModel>();
	RW_ASSERT(mA != NULL);
	RW_ASSERT(mB != NULL);
	if (!data.isInitialized())
		data.setSpecificData(new PQPData());
	PQPData* const pqpData = dynamic_cast<PQPData*>(data.getSpecificData());
	RW_ASSERT(pqpData);
	if (!tracking.isInitialized())
		tracking.setStrategyData(new PQPTracking());
	PQPTracking* const pqpTracking = dynamic_cast<PQPTracking*>(tracking.getStrategyData());
	RW_ASSERT(pqpTracking);

	std::vector<PQPTracking::TrackInfo> newInfo;

	const Transform3D<> aTb = inverse(wTa)*wTb;
	const EAA<> dif(inverse(pqpTracking->aTb.R())*aTb.R());
	const double absoluteThreshold = getUpdateThresholdAbsolute();
	const double linThreshold = getUpdateThresholdLinear()*(pqpTracking->aTb.P()-aTb.P()).norm2();
	const double angThreshold = getUpdateThresholdAngular()*dif.angle();
	const double threshold = absoluteThreshold+linThreshold+angThreshold;

	std::vector<PQPTracking::TrackInfo>::iterator it;
	for (it = pqpTracking->info.begin(); it != pqpTracking->info.end(); it++) {
		const PQPTracking::TrackInfo& info = *it;
		std::vector<PQPTracking::TrackInfo> currentInfos;
		currentInfos.push_back(*it);
		for (std::size_t i = 0; i < info.total-1; i++) {
			it++;
			currentInfos.push_back(*it);
		}
		const Model modelA = mA->models[info.modelIDa];
		const Model modelB = mB->models[info.modelIDb];

		std::vector<Contact> tmpContacts;
		findContact(tmpContacts, modelA, wTa, modelB, wTb, pqpData, false);
		BOOST_FOREACH(Contact &c, tmpContacts) {
			c.setModelA(mA);
			c.setModelB(mB);
		}

		if (_filtering == MANIFOLD) {
			tmpContacts = manifoldFilter(tmpContacts);
		}

		BOOST_FOREACH(Contact &c, tmpContacts) {
			c.setDepth(getThreshold()+c.getDepth());
		}

		std::size_t curId = 0;
		BOOST_FOREACH(const PQPTracking::TrackInfo& info, currentInfos) {
			const Vector3D<> posA = info.posA;
			const Vector3D<> posB = info.posB;
			std::vector<std::size_t> candidates;
			for (std::size_t k = 0; k < tmpContacts.size(); k++) {
				const Contact& c = tmpContacts[k];
				const Vector3D<> newPosA = inverse(wTa)*c.getPointA();
				const Vector3D<> newPosB = inverse(wTb)*c.getPointB();
				if ((newPosA-posA).norm2() <= threshold && (newPosB-posB).norm2() <= threshold) {
					candidates.push_back(k);
				}
			}
			if (candidates.size() > 0) {
				double depth = tmpContacts[candidates[0]].getDepth();
				std::size_t deepest = candidates[0];
				for (std::size_t k = 1; k < candidates.size(); k++) {
					double tmpDepth = tmpContacts[candidates[k]].getDepth();
					if (tmpDepth > depth) {
						depth = tmpDepth;
						deepest = candidates[k];
					}
				}
				contacts.push_back(tmpContacts[deepest]);
				newInfo.push_back(info);
				newInfo.back().posA = inverse(wTa)*tmpContacts[deepest].getPointA();
				newInfo.back().posB = inverse(wTb)*tmpContacts[deepest].getPointB();
				newInfo.back().id = curId;
				curId++;
				tmpContacts.erase(tmpContacts.begin()+deepest);
			}
		}
		for (std::size_t i = 0; i < curId; i++) {
			newInfo[newInfo.size()-1-i].total = curId;
		}
	}
	pqpTracking->aTb = inverse(wTa)*wTb;
	pqpTracking->info = newInfo;
	return contacts;
}

std::string ContactStrategyPQP::getName() {
	return "ContactStrategyPQP";
}

ProximityModel::Ptr ContactStrategyPQP::createModel() {
	TriMeshModel* model = new TriMeshModel(this);
	return ownedPtr(model);
}

void ContactStrategyPQP::destroyModel(ProximityModel* model) {
	TriMeshModel* bmodel = dynamic_cast<TriMeshModel*>(model);
	RW_ASSERT(bmodel);
	BOOST_FOREACH(const Model& model, bmodel->models) {
		removeGeometry(bmodel,model.geoId);
	}
	bmodel->models.clear();
}

bool ContactStrategyPQP::addGeometry(ProximityModel* model, const Geometry& geom) {
	TriMeshModel* bmodel = dynamic_cast<TriMeshModel*>(model);
	RW_ASSERT(bmodel);
	GeometryData::Ptr geomData = geom.getGeometryData();
	if (!_matchAll) {
		if (geomData->getType() != GeometryData::LineMesh &&
				geomData->getType() != GeometryData::PlainTriMesh &&
				geomData->getType() != GeometryData::IdxTriMesh)
			return false;
	}
	// todo: Convex decomposition when trimesh is concave
	TriMesh* sData = (TriMesh*) geomData.get();
	Model newModel;
	newModel.geoId = geom.getId();
	newModel.transform = geom.getTransform();
	newModel.mesh = sData->getTriMesh();
	newModel.frame = geom.getFrame();
	newModel.pmodel = _narrowStrategy->createModel();
	bmodel->models.push_back(newModel);

	_narrowStrategy->addGeometry(newModel.pmodel.get(),geom);

	return true;
}

bool ContactStrategyPQP::addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy) {
	return addGeometry(model, *geom);
}

bool ContactStrategyPQP::removeGeometry(ProximityModel* model, const std::string& geomId) {
	TriMeshModel* bmodel = dynamic_cast<TriMeshModel*>(model);
	RW_ASSERT(bmodel);
	for (std::vector<Model>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++) {
		if ((*it).geoId == geomId) {
			_narrowStrategy->removeGeometry((*it).pmodel.get(), geomId);
			bmodel->models.erase(it);
			return true;
		}
	}
	return false;
}

std::vector<std::string> ContactStrategyPQP::getGeometryIDs(ProximityModel* model) {
	TriMeshModel* bmodel = dynamic_cast<TriMeshModel*>(model);
	RW_ASSERT(bmodel);
	std::vector<std::string> res;
	for (std::vector<Model>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++)
		res.push_back((*it).geoId);
	return res;
}

void ContactStrategyPQP::clear() {
	// Nothing to clear
}

void ContactStrategyPQP::setMatchAll(bool matchAll) {
	_matchAll = matchAll;
}

void ContactStrategyPQP::setContactFilter(ContactFilter filter) {
	_filtering = filter;
}

std::vector<Contact> ContactStrategyPQP::manifoldFilter(const std::vector<Contact> &contacts) {
	std::vector<Contact> res;

	std::vector<ContactPoint> rwcontacts(contacts.size());
	int* srcIdx = new int[contacts.size()];
	int* dstIdx = new int[contacts.size()];
	std::vector<ContactPoint> rwClusteredContacts(contacts.size());

	for (std::size_t i = 0; i < contacts.size(); i++) {
		ContactPoint &point = rwcontacts[i];
		point.n = normalize( contacts[i].getNormal() );
		point.p = (contacts[i].getPointA()+contacts[i].getPointB())/2.;
		point.penetration = contacts[i].getDepth();
        point.userdata = (void*) &(contacts[i]);
	}

	int fnumc = ContactCluster::normalThresClustering(
			&rwcontacts[0], (int)contacts.size(),
			&srcIdx[0], &dstIdx[0],
			&rwClusteredContacts[0],
			10*Deg2Rad);

	std::vector<ContactPoint> &src = rwcontacts;
	std::vector<ContactPoint> &dst = rwClusteredContacts;

	std::vector< OBRManifold > manifolds;
	// for each cluster we fit a manifold
	for(int i=0;i<fnumc;i++){
		int idxFrom = srcIdx[i];
		const int idxTo = srcIdx[i+1];
		// locate the manifold that idxFrom is located in
		OBRManifold manifold(15*Deg2Rad,0.2);
		for(;idxFrom<idxTo; idxFrom++){
			ContactPoint &point = src[dstIdx[idxFrom]];
			manifold.addPoint(point);
		}
		manifolds.push_back(manifold);
	}

	// run through all manifolds and get the contact points that will be used.
	int contactIdx = 0;
	BOOST_FOREACH(OBRManifold& obr, manifolds){
		const int nrContacts = obr.getNrOfContacts();
		// if the manifold area is very small then we only use a single point
		// for contact
		const Vector3D<> hf = obr.getHalfLengths();
		if(hf(0)*hf(1)<(0.001*0.001)){
			RW_ASSERT( contactIdx<(int)dst.size() );
			dst[contactIdx] = obr.getDeepestPoint();
			contactIdx++;
		} else {
			for(int j=0;j<nrContacts; j++){
				RW_ASSERT( contactIdx<(int)dst.size() );
				dst[contactIdx] = obr.getContact(j);
				contactIdx++;
			}
		}
	}

	// Run through all contacts and define contact constraints between them
	std::vector<ContactPoint> &contactPointList = dst;
	for (int i = 0; i < contactIdx; i++) {
		ContactPoint *point = &contactPointList[i];
		point->n = normalize(point->n);
		Contact &con = *((Contact*)point->userdata);

		const double rwnlength = MetricUtil::norm2(point->n);
		if((0.9>rwnlength) || (rwnlength>1.1)){
			continue;
		}
		con.setNormal(point->n);
		con.setPointA(point->p-point->n*fabs(point->penetration/2));
		con.setPointB(point->p+point->n*fabs(point->penetration/2));
		con.setDepth(point->penetration);
		res.push_back(con);
	}

	delete[] srcIdx;
	delete[] dstIdx;

	return res;
}

double ContactStrategyPQP::getThreshold() const {
	double threshold = _propertyMap.get<double>("ContactStrategyPQPThreshold", -0.1);
	if (threshold == -0.1)
		threshold = _propertyMap.get<double>("MaxSepDistance", 0.0005);
    return threshold;
}

double ContactStrategyPQP::getUpdateThresholdAbsolute() const {
	return _propertyMap.get<double>("ContactStrategyPQPUpdateThresholdAbsolute", 0.001);
}

double ContactStrategyPQP::getUpdateThresholdLinear() const {
	return _propertyMap.get<double>("ContactStrategyPQPUpdateThresholdLinear", 2.);
}

double ContactStrategyPQP::getUpdateThresholdAngular() const {
	return _propertyMap.get<double>("ContactStrategyPQPUpdateThresholdAngular", 0.25);
}
