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

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/dynamics/OBRManifold.hpp>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::proximity;
using namespace rw::math;
using namespace rwlibs::proximitystrategies;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;

ContactStrategyPQP::ContactStrategyPQP():
	_matchAll(true),
	_narrowStrategy(new ProximityStrategyPQP()),
	_filtering(MANIFOLD)
{
}

ContactStrategyPQP::~ContactStrategyPQP() {
	delete _narrowStrategy;
}

bool ContactStrategyPQP::match(GeometryData::Ptr geoA, GeometryData::Ptr geoB) {
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

std::vector<Contact> ContactStrategyPQP::findContacts(ProximityModel* a, const Transform3D<>& wTa, ProximityModel* b, const Transform3D<>& wTb) {
	ContactStrategyData data;
	return findContacts(a,wTa,b,wTb,data);
}

std::vector<Contact> ContactStrategyPQP::findContacts(ProximityModel* a, const Transform3D<>& wTa, ProximityModel* b, const Transform3D<>& wTb, ContactStrategyData &data) {
	std::vector<Contact> contacts;
	TriMeshModel* mA = (TriMeshModel*) a;
	TriMeshModel* mB = (TriMeshModel*) b;
	for (std::vector<Model>::iterator itA = mA->models.begin(); itA < mA->models.end(); itA++) {
		for (std::vector<Model>::iterator itB = mB->models.begin(); itB < mB->models.end(); itB++) {
			Model modelA = *itA;
			Model modelB = *itB;

			MultiDistanceResult *res;
		    ProximityStrategyData data;

			data.setCollisionQueryType(CollisionStrategy::AllContacts);
			res = &_narrowStrategy->distances(mA->pmodel, wTa, mB->pmodel, wTb, getThreshold(), data);

			for(size_t i=0;i<res->distances.size();i++){
				Contact c;
				Vector3D<> p1 = wTa*modelA.transform*res->p1s[i];
				Vector3D<> p2 = wTa*modelA.transform*res->p2s[i];
				c.setPointA(p1);
				c.setPointB(p2);

				c.setFrameA(modelA.frame);
				c.setFrameB(modelB.frame);

				if(res->distances[i]<0.00000001){
					std::pair< Vector3D<>, Vector3D<> > normals = _narrowStrategy->getSurfaceNormals(*res, i);
					// the second is described in b's refframe so convert both to world and combine them
					Vector3D<> a_normal = wTa.R() * normals.first;
					Vector3D<> b_normal = wTb.R() * normals.second;

					c.setNormal(-normalize( a_normal - b_normal ));
				} else {
					c.setNormal(normalize(p2-p1));
				}
				c.setDepth(-res->distances[i]);
				c.setModelA(mA);
				c.setModelB(mB);
				c.setTransform(Transform3D<>::identity()); // Should be set correctly!
				contacts.push_back(c);
			}

			res->clear();
		}
	}
	switch (_filtering) {
	case MANIFOLD:
		return manifoldFilter(contacts);
	default:
		return contacts;
	}
}

std::string ContactStrategyPQP::getName() {
	return "ContactStrategyPQP";
}

ProximityModel::Ptr ContactStrategyPQP::createModel() {
	TriMeshModel* model = new TriMeshModel(this);
	model->pmodel = _narrowStrategy->createModel();
	return ownedPtr(model);
}

void ContactStrategyPQP::destroyModel(ProximityModel* model) {
	TriMeshModel* bmodel = (TriMeshModel*) model;
	_narrowStrategy->destroyModel(bmodel->pmodel.get());
	bmodel->models.clear();
}

bool ContactStrategyPQP::addGeometry(ProximityModel* model, const Geometry& geom) {
	TriMeshModel* bmodel = (TriMeshModel*) model;
	GeometryData::Ptr geomData = geom.getGeometryData();
	if (!_matchAll) {
		if (geomData->getType() != GeometryData::LineMesh &&
				geomData->getType() != GeometryData::PlainTriMesh &&
				geomData->getType() != GeometryData::IdxTriMesh)
			return false;
	}
	TriMesh* sData = (TriMesh*) geomData.get();
	Model newModel;
	newModel.geoId = geom.getId();
	newModel.transform = geom.getTransform();
	newModel.mesh = sData->getTriMesh();
	newModel.frame = geom.getFrame();
	bmodel->models.push_back(newModel);

	_narrowStrategy->addGeometry(bmodel->pmodel.get(),geom);

	return true;
}

bool ContactStrategyPQP::addGeometry(ProximityModel* model, Geometry::Ptr geom, bool forceCopy) {
	return addGeometry(model, *geom);
}

bool ContactStrategyPQP::removeGeometry(ProximityModel* model, const std::string& geomId) {
	TriMeshModel* bmodel = (TriMeshModel*) model;
	for (std::vector<Model>::iterator it = bmodel->models.begin(); it < bmodel->models.end(); it++) {
		if ((*it).geoId == geomId) {
			bmodel->models.erase(it);
			return true;
		}
	}
	_narrowStrategy->removeGeometry(bmodel->pmodel.get(), geomId);
	return false;
}

std::vector<std::string> ContactStrategyPQP::getGeometryIDs(ProximityModel* model) {
	TriMeshModel* bmodel = (TriMeshModel*) model;
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
	std::vector<ContactPoint> cpRes;

	std::vector<ContactPoint> rwcontacts(contacts.size());
	int srcIdx[contacts.size()];
	int dstIdx[contacts.size()];
	std::vector<ContactPoint> rwClusteredContacts(contacts.size());

	for (std::size_t i = 0; i < contacts.size(); i++) {
		ContactPoint &point = rwcontacts[i];
		point.n = normalize( contacts[i].getNormal() );
		point.p = contacts[i].getNormal()*(contacts[i].getDepth()/2) + contacts[i].getPointA();
		point.penetration = contacts[i].getDepth();
        point.userdata = (void*) &(contacts[i]);
	}

	int fnumc = ContactCluster::normalThresClustering(
			&rwcontacts[0], contacts.size(),
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
	rw::math::Vector3D<> contactNormalAvg(0,0,0);
	BOOST_FOREACH(OBRManifold& obr, manifolds){
		contactNormalAvg += obr.getNormal();
		int nrContacts = obr.getNrOfContacts();
		// if the manifold area is very small then we only use a single point
		// for contact
		Vector3D<> hf = obr.getHalfLengths();
		if(hf(0)*hf(1)<(0.001*0.001)){
			cpRes.push_back( obr.getDeepestPoint() );
			RW_ASSERT( contactIdx<(int)dst.size() );
			dst[contactIdx] = obr.getDeepestPoint();
			contactIdx++;
		} else {
			//std::cout << "Manifold: " << nrContacts << ";" << std::endl;
			for(int j=0;j<nrContacts; j++){
				cpRes.push_back( obr.getContact(j) );
				RW_ASSERT( contactIdx<(int)dst.size() );
				dst[contactIdx] = obr.getContact(j);
				contactIdx++;
			}
		}
	}

	Vector3D<> cNormal(0,0,0);
	// Run through all contacts and define contact constraints between them
	std::vector<ContactPoint> &contactPointList = dst;
	int num = contactIdx;

	for (int i = 0; i < num; i++) {
		ContactPoint *point = &contactPointList[i];
		point->n = normalize(point->n);
		Contact &con = *((Contact*)point->userdata);

		cNormal += point->n;
		double rwnlength = MetricUtil::norm2(point->n);
		if((0.9>rwnlength) || (rwnlength>1.1)){
			//std::cout <<  "\n\n Normal not normalized _0_ !\n"<<std::endl;
			continue;
		}
		con.setNormal(point->n);
		con.setPointA(point->p-point->n*point->penetration/2);
		con.setPointB(point->p+point->n*point->penetration/2);
		con.setDepth(point->penetration);
		res.push_back(con);
	}

	return res;
}

double ContactStrategyPQP::getThreshold() const {
	double threshold = _propertyMap.get<double>("ContactStrategyPQPThreshold", -0.1);
	if (threshold == -0.1)
		threshold = _propertyMap.get<double>("MaxSepDistance", 0.0005);
    return threshold;
}
