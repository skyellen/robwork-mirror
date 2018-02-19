/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BREP.hpp"
#include "Curve.hpp"
#include "Face.hpp"
#include "GenericFace.hpp"
#include "Surface.hpp"
#include "Shell.hpp"

#include <rw/geometry/PlainTriMesh.hpp>

#include <list>

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

namespace {
class BREPShell: public Shell {
public:
	BREPShell(const BREP* brep): Shell(), _brep(brep) {}
	~BREPShell() {}
	GeometryType getType() const { return _brep->getType(); }
	bool isConvex() { return _brep->isConvex(); }
	std::size_t size() const { return _brep->faceCount(); }

private:
	Face::CPtr doGetFace(std::size_t idx) const {
		const GenericFace::Ptr bface = ownedPtr(new GenericFace());
		bface->setSurface(_brep->getSurface(idx));
		bface->setVertices(_brep->getVertices(idx));
		const std::vector<Curve::Ptr> curves = _brep->getCurves(idx);
		for (std::size_t i = 0; i < curves.size(); i++) {
			bface->setCurve(i,curves[i]);
		}
		return bface;
	}

	const BREP* const _brep;
};
}

BREP::BREP() {
	setMeshResolution();
}

BREP::~BREP() {
	for (std::size_t i = 0; i < _faces.size(); i++) {
		delete _faces[i];
	}
	for (std::size_t i = 0; i < _edges.size(); i++) {
		delete _edges[i].first;
		delete _edges[i].second;
	}
	for (std::size_t i = 0; i < _vertices.size(); i++)
		delete _vertices[i];
	_faces.clear();
	_edges.clear();
	_vertices.clear();
}

TriMesh::Ptr BREP::getTriMesh(bool) {
	const PlainTriMeshN1D::Ptr mesh = ownedPtr(new PlainTriMeshN1D());
	for (std::size_t i = 0; i < _faces.size(); i++) {
		const PlainTriMeshN1D::Ptr faceMesh = faceTriMesh(i).scast<PlainTriMeshN1D>();
		mesh->add(faceMesh);
	}
	return mesh;
}

bool BREP::isConvex() {
	return false;
}

bool BREP::isConvex() const {
	return false;
}

std::size_t BREP::faceCount() const {
	std::size_t cnt = 0;
	for (std::size_t i = 0; i < _faces.size(); i++) {
		if (_faces[i]->surfaceSet)
			cnt++;
	}
	return cnt;
}

std::vector<Vector3D<> > BREP::getVertices(std::size_t loopIdx) const {
	std::vector<Vector3D<> > vertices;
	const HalfEdge* edge = _faces[loopIdx]->edge;
	do {
		vertices.push_back(edge->previousVertex->point);
		edge = edge->nextEdge;
	} while (edge != _faces[loopIdx]->edge);
	return vertices;
}

std::vector<Curve::Ptr> BREP::getCurves(std::size_t loopIdx) const {
	std::vector<Curve::Ptr> curves;
	const HalfEdge* half = _faces[loopIdx]->edge;
	do {
		if (half->reversed)
			curves.push_back(getCurve(half->curveIndex).reverse());
		else
			curves.push_back(getCurve(half->curveIndex).clone());
		half = half->nextEdge;
	} while (half != _faces[loopIdx]->edge);
	return curves;
}

std::size_t BREP::getSurfaceIndex(std::size_t loop) {
	std::size_t index = _faces.size();
	if (_faces[loop]->surfaceSet)
		index = _faces[loop]->surfaceIndex;
	return index;
}

OBB<> BREP::obb(const rw::math::Rotation3D<>& R) {
	std::list<Curve::CPtr> edges;
	for (std::size_t i = 0; i < _edges.size(); i++) {
		edges.push_back(getCurve(_edges[i].first->curveIndex).clone());
	}

	const Vector3D<> e1 = R.getCol(0);
	const Vector3D<> e2 = R.getCol(1);
	const Vector3D<> e3 = R.getCol(2);

	std::vector<double> min(3,std::numeric_limits<double>::max());
	std::vector<double> max(3,-std::numeric_limits<double>::max());
	for (std::list<Curve::CPtr>::const_iterator it = edges.begin(); it != edges.end(); it++) {
		for (std::size_t i = 0; i < 3; i++) {
			const std::pair<double,double> extremums = (*it)->extremums(R.getCol(i));
			if (extremums.first < min[i])
				min[i] = extremums.first;
			if (extremums.second > max[i])
				max[i] = extremums.second;
		}
	}

	// Deal with surfaces
	for (std::vector<Face*>::const_iterator it = _faces.begin(); it != _faces.end(); it++) {
		for (std::size_t i = 0; i < 3; i++) {
			const std::pair<double,double> extremums = getSurface((*it)->surfaceIndex).extremums(R.getCol(i));
			if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min[i])
				min[i] = extremums.first;
			if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max[i])
				max[i] = extremums.second;
		}
	}

	const Vector3D<> P = Vector3D<>((max[0]+min[0])*e1+(max[1]+min[1])*e2+(max[2]+min[2])*e3)/2;
	const Vector3D<> halfLen = Vector3D<>(max[0]-min[0],max[1]-min[1],max[2]-min[2])/2;
	return OBB<>(Transform3D<>(P,R), halfLen);
}

OBB<> BREP::obb() {
	const TriMesh::Ptr mesh = getTriMesh();
	const OBB<> obbFit = OBB<>::buildTightOBB(*mesh);
	return obb(obbFit.getTransform().R());
}
/*
const std::vector<BREP::Vertex*>& BREP::vertices() const {
	return _vertices;
}

const std::vector<std::pair<BREP::HalfEdge*, BREP::HalfEdge*> >& BREP::edges() const {
	return _edges;
}
*/

Shell::CPtr BREP::doShellProxyBREP() const {
	return ownedPtr(new BREPShell(this));
}
/*
std::vector<const BREP::HalfEdge*> BREP::loop(const Face& face) const {
	std::vector<const HalfEdge*> loop;
	const HalfEdge* edge = face.edge;
	if (edge == NULL)
		return loop;
	loop.push_back(edge);
	edge = edge->nextEdge;
	while (edge != NULL && edge != face.edge) {
		loop.push_back(edge);
		edge = edge->nextEdge;
	}
	return loop;
}
*/

class BREP::CommonCurveSetGeneric: public BREP::CommonCurveSet {
public:
	CommonCurveSetGeneric(const BREP* brep, const std::vector<const BREP::HalfEdge*>& edges): _brep(brep), _edges(edges) {}
	virtual ~CommonCurveSetGeneric() {}
	virtual std::size_t size() const { return _edges.size(); }
	virtual const Curve& curve(std::size_t index) const { return _brep->getCurve(_edges[index]->curveIndex); }
	virtual const Surface& surfaceLeft(std::size_t index) const {
		return _brep->getSurface(_edges[index]->face->surfaceIndex);
	}
	virtual const Surface& surfaceRight(std::size_t index) const {
		return _brep->getSurface(_edges[index]->oppositeEdge->face->surfaceIndex);
	}
private:
	const BREP* const _brep;
	const std::vector<const BREP::HalfEdge*> _edges;
};

BREP::CommonCurveSet::CPtr BREP::getCommonCurves(const std::set<std::size_t>& faces) const {
	std::vector<const HalfEdge*> edges;
	for (std::set<std::size_t>::const_iterator it = faces.begin(); it != faces.end(); it++) {
		const HalfEdge* edge = _faces[*it]->edge;
		if (edge == NULL)
			continue;
		do {
			std::set<std::size_t>::const_iterator itB = it;
			for (itB++; itB != faces.end(); itB++) {
				const HalfEdge* edgeB = _faces[*itB]->edge;
				if (edgeB == NULL)
					continue;
				do {
					if (edge->oppositeEdge == edgeB) {
						//curves.push_back(edge->curve);
						edges.push_back(edge);
					}
					edgeB = edgeB->nextEdge;
				} while (edgeB != NULL && edgeB != _faces[*itB]->edge);
			}
			edge = edge->nextEdge;
		} while (edge != NULL && edge != _faces[*it]->edge);
	}
	return ownedPtr(new CommonCurveSetGeneric(this, edges));
}

void BREP::addVertex(const Vector3D<>& point) {
	_vertices.push_back(new Vertex(point));
}

void BREP::makeLoop(int singleEdgeId) {
	RW_ASSERT(singleEdgeId != 0);
	HalfEdge* const e = (singleEdgeId < 0) ? _edges[-singleEdgeId-1].second : _edges[singleEdgeId-1].first;
	RW_ASSERT(e->previousVertex == e->nextVertex);
	RW_ASSERT(e->oppositeEdge->previousVertex == e->oppositeEdge->nextVertex);
	RW_ASSERT(e->previousVertex == e->oppositeEdge->nextVertex);

	if (e->nextEdge != e) {
		HalfEdge* const prev = e->previousEdge;
		HalfEdge* const next = e->nextEdge;
		if (prev == e->oppositeEdge || next == e->oppositeEdge) {
			prev->nextEdge = next;
			next->previousEdge = prev;

			e->nextEdge = e;
			e->previousEdge = e;
		} else {
			RW_THROW("Could not figure out how to make single edge loop!");
		}
	}

	if (e->face == NULL) {
		_faces.push_back(new Face());
		_faces.back()->edge = e;
		e->face = _faces.back();
	}
}

void BREP::setEdgeOrder(int before, int after) {
	RW_ASSERT(before != 0);
	RW_ASSERT(after != 0);
	HalfEdge* const e1 = (before < 0) ? _edges[-before-1].second : _edges[before-1].first;
	HalfEdge* const e2 = (after < 0) ? _edges[-after-1].second : _edges[after-1].first;
	setHalfEdgeOrder(e1,e2);
	if (e2->face == NULL) {
		_faces.push_back(new Face());
		_faces.back()->edge = e2;
		e2->face = _faces.back();
	}
	e1->face = e2->face;
}

void BREP::setHalfEdgeOrder(HalfEdge* before, HalfEdge* after) {
	if (before->nextEdge == after)
		return;

	if (before == NULL)
		RW_THROW("before is null!");
	if (after == NULL)
		RW_THROW("after is null!");

	HalfEdge* const befNext = before->nextEdge;
	HalfEdge* const aftPrev = after->previousEdge;

	if (befNext == NULL)
		RW_THROW("befNext is null!");
	if (aftPrev == NULL)
		RW_THROW("aftPrev is null!");

	/*if (after->nextEdge == after) {
		before->nextEdge = after;
		after->previousEdge = before;

		after->nextEdge = befNext;
		befNext->previousEdge = after;
	} else {*/
		HalfEdge* const free = freeHalfEdge(*after->previousVertex, after, before);
		if (free == NULL) {
			RW_THROW("No free half edge. Can not make loop.");
		}
		HalfEdge* const freeNext = free->nextEdge;

		before->nextEdge = after;
		after->previousEdge = before;

		if (free != before) {
			free->nextEdge = befNext;
			befNext->previousEdge = free;
		}

		if (freeNext != after) {
			aftPrev->nextEdge = freeNext;
			freeNext->previousEdge = aftPrev;
		}
	//}
}

BREP::HalfEdge* BREP::freeHalfEdge(const Vertex& endVertex, const HalfEdge* from, const HalfEdge* to) const {
	const HalfEdge* edge = (from == NULL) ? endVertex.nextEdge : from;
	while(edge != NULL) {
		//if (edge->previousVertex == edge->nextVertex && from != NULL) {
		//	edge = edge->nextEdge;
		//} else {
			HalfEdge* const oppsosite = edge->oppositeEdge;
			if (oppsosite->face == NULL)
				return oppsosite;
			edge = oppsosite->nextEdge;
		//}
	}
	return NULL;
}

OBB<> BREP::faceOBB(std::size_t faceIndex) {
	std::size_t cnt = 0;
	for (std::size_t i = 0; i < _faces.size() && cnt < faceIndex+1; i++) {
		if (_faces[i]->surfaceSet)
			cnt++;
	}
	cnt--;

	std::list<Curve::CPtr> edges;
	const HalfEdge* edge = _faces[cnt]->edge;
	do {
		edges.push_back(getCurve(edge->curveIndex).clone());
		edge = edge->nextEdge;
	} while (edge != _faces[cnt]->edge);

	const TriMesh::Ptr mesh = faceTriMesh(faceIndex);
	const OBB<> obb = OBB<>::buildTightOBB(*mesh);

	const Rotation3D<> R = obb.getTransform().R();
	const Vector3D<> e1 = R.getCol(0);
	const Vector3D<> e2 = R.getCol(1);
	const Vector3D<> e3 = R.getCol(2);

	std::vector<double> min(3,std::numeric_limits<double>::max());
	std::vector<double> max(3,-std::numeric_limits<double>::max());
	for (std::list<Curve::CPtr>::const_iterator it = edges.begin(); it != edges.end(); it++) {
		for (std::size_t i = 0; i < 3; i++) {
			const std::pair<double,double> extremums = (*it)->extremums(R.getCol(i));
			//std::cout << "edge: " << i << " " << extremums.first << " " << extremums.second << std::endl;
			if (extremums.first < min[i])
				min[i] = extremums.first;
			if (extremums.second > max[i])
				max[i] = extremums.second;
		}
	}

	// Deal with surface itself
	for (std::size_t i = 0; i < 3; i++) {
		const std::pair<double,double> extremums = getSurface(_faces[cnt]->surfaceIndex).extremums(R.getCol(i));
		if (extremums.first != -std::numeric_limits<double>::max() && extremums.first < min[i])
			min[i] = extremums.first;
		if (extremums.second != std::numeric_limits<double>::max() && extremums.second > max[i])
			max[i] = extremums.second;
	}

	const Vector3D<> P = Vector3D<>((max[0]+min[0])*e1+(max[1]+min[1])*e2+(max[2]+min[2])*e3)/2;
	const Vector3D<> halfLen = Vector3D<>(max[0]-min[0],max[1]-min[1],max[2]-min[2])/2;
	return OBB<>(Transform3D<>(P,R), halfLen);
}

OBB<> BREP::edgeOBR(std::size_t edge) const {
	return getCurve(_edges[edge].first->curveIndex).obr();
}

std::pair<double,double> BREP::faceExtremums(std::size_t faceIndex, const Vector3D<>& dir) const {
	std::pair<double,double> res(std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

	double& min = res.first;
	double& max = res.second;

	std::size_t cnt = 0;
	for (std::size_t i = 0; i < _faces.size() && cnt < faceIndex+1; i++) {
		if (_faces[i]->surfaceSet)
			cnt++;
	}
	cnt--;

	std::list<Curve::CPtr> edges;
	const HalfEdge* edge = _faces[cnt]->edge;
	do {
		edges.push_back(getCurve(edge->curveIndex).clone());
		edge = edge->nextEdge;
	} while (edge != _faces[cnt]->edge);

	// Check edges
	for (std::list<Curve::CPtr>::const_iterator it = edges.begin(); it != edges.end(); it++) {
		const std::pair<double,double> extremums = (*it)->extremums(dir);
		if (extremums.first < min)
			min = extremums.first;
		if (extremums.second > max)
			max = extremums.second;
	}

	// Deal with surface itself
	const std::pair<double,double> extremums = getSurface(_faces[cnt]->surfaceIndex).extremums(dir);
	if (extremums.first != -std::numeric_limits<double>::max())
		min = extremums.first;
	if (extremums.second != std::numeric_limits<double>::max())
		max = extremums.second;

	return res;
}

TriMesh::Ptr BREP::faceTriMesh(std::size_t faceIndex) {
	std::size_t cnt = 0;
	for (std::size_t i = 0; i < _faces.size() && cnt < faceIndex+1; i++) {
		if (_faces[i]->surfaceSet)
			cnt++;
	}
	cnt--;

	// Get the halfedges
	std::list<const HalfEdge*> edges;
	const HalfEdge* edge = _faces[cnt]->edge;
	do {
		edges.push_back(edge);
		edge = edge->nextEdge;
	} while (edge != _faces[cnt]->edge);

	// Construct loop polygon
	std::vector<Vector3D<> > polygon3d;
	for (std::list<const HalfEdge*>::const_iterator it = edges.begin(); it != edges.end(); it++) {
		const Curve& c = getCurve((*it)->curveIndex);
		const std::list<Vector3D<> > points = c.discretizeAdaptive(_resolution);
		if (!(*it)->reversed)
			polygon3d.insert(polygon3d.end(),points.begin(),--points.end());
		else
			polygon3d.insert(polygon3d.end(),points.rbegin(),--points.rend());
	}

	const Surface::Ptr surface = getSurface(_faces[cnt]->surfaceIndex).clone();
	surface->setDiscretizationResolution(_resolution);
	return surface->getTriMesh(polygon3d);
}

void BREP::copyTopologyTo(BREP::Ptr brep) const {
	brep->_vertices.resize(_vertices.size());
	brep->_edges.resize(_edges.size());
	brep->_faces.resize(_faces.size());

	for (std::size_t i = 0; i < _vertices.size(); i++) {
		brep->_vertices[i] = new Vertex(_vertices[i]->point);
	}
	for (std::size_t i = 0; i < _edges.size(); i++) {
		std::size_t v1 = _vertices.size();
		std::size_t v2 = _vertices.size();
		for (std::size_t vi = 0; vi < _vertices.size(); vi++) {
			if (_edges[i].first->previousVertex == _vertices[vi])
				v1 = vi;
			if (_edges[i].first->nextVertex == _vertices[vi])
				v2 = vi;
		}
		if (v1 >= _vertices.size() || v2 >= _vertices.size())
			RW_THROW("Make sure BREP is valid before scaling it.");
		std::pair<HalfEdge*, HalfEdge*>& edge = brep->_edges[i];
		edge.first = new HalfEdge(_edges[i].first->curveIndex);
		edge.second = new HalfEdge(_edges[i].second->curveIndex);
		edge.first->previousVertex = brep->_vertices[v1];
		edge.first->nextVertex = brep->_vertices[v2];
		edge.first->oppositeEdge = edge.second;
		edge.first->reversed = _edges[i].first->reversed;
		edge.second->previousVertex = brep->_vertices[v2];
		edge.second->nextVertex = brep->_vertices[v1];
		edge.second->oppositeEdge = edge.first;
		edge.second->reversed = _edges[i].second->reversed;
	}
	for (std::size_t i = 0; i < _vertices.size(); i++) {
		for (std::size_t nei = 0; nei < _edges.size() && brep->_vertices[i]->nextEdge == NULL; nei++) {
			if (_edges[nei].first == _vertices[i]->nextEdge)
				brep->_vertices[i]->nextEdge = brep->_edges[nei].first;
			else if (_edges[nei].second == _vertices[i]->nextEdge)
				brep->_vertices[i]->nextEdge = brep->_edges[nei].second;
		}
		if (brep->_vertices[i]->nextEdge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
	}
	//std::cout << "adding faces" << std::endl;
	// Make the loops...
	for (std::size_t i = 0; i < _faces.size(); i++) {
		brep->_faces[i] = new Face();
		brep->_faces[i]->surfaceIndex = _faces[i]->surfaceIndex;
		brep->_faces[i]->surfaceSet = _faces[i]->surfaceSet;
		for (std::size_t nei = 0; nei < _edges.size() && brep->_faces[i]->edge == NULL; nei++) {
			if (_edges[nei].first == _faces[i]->edge)
				brep->_faces[i]->edge = brep->_edges[nei].first;
			else if (_edges[nei].second == _faces[i]->edge)
				brep->_faces[i]->edge = brep->_edges[nei].second;
		}
		if (brep->_faces[i]->edge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
	}
	//std::cout << "adding face and edge pointers to edges" << std::endl;
	for (std::size_t i = 0; i < _edges.size(); i++) {
		for (std::size_t fi = 0; fi < _faces.size() && (brep->_edges[i].first->face == NULL || brep->_edges[i].second->face == NULL); fi++) {
			if (_faces[fi] == _edges[i].first->face)
				brep->_edges[i].first->face = brep->_faces[fi];
			if (_faces[fi] == _edges[i].second->face)
				brep->_edges[i].second->face = brep->_faces[fi];
		}
		if (brep->_edges[i].first->face == NULL || brep->_edges[i].second->face == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");

		for (std::size_t nei = 0; nei < _edges.size() && brep->_edges[i].first->nextEdge == NULL; nei++) {
			if (_edges[nei].first == _edges[i].first->nextEdge)
				brep->_edges[i].first->nextEdge = brep->_edges[nei].first;
			else if (_edges[nei].second == _edges[i].first->nextEdge)
				brep->_edges[i].first->nextEdge = brep->_edges[nei].second;
		}
		for (std::size_t nei = 0; nei < _edges.size() && brep->_edges[i].first->previousEdge == NULL; nei++) {
			if (_edges[nei].first == _edges[i].first->previousEdge)
				brep->_edges[i].first->previousEdge = brep->_edges[nei].first;
			else if (_edges[nei].second == _edges[i].first->previousEdge)
				brep->_edges[i].first->previousEdge = brep->_edges[nei].second;
		}
		for (std::size_t nei = 0; nei < _edges.size() && brep->_edges[i].second->nextEdge == NULL; nei++) {
			if (_edges[nei].first == _edges[i].second->nextEdge)
				brep->_edges[i].second->nextEdge = brep->_edges[nei].first;
			else if (_edges[nei].second == _edges[i].second->nextEdge)
				brep->_edges[i].second->nextEdge = brep->_edges[nei].second;
		}
		for (std::size_t nei = 0; nei < _edges.size() && brep->_edges[i].second->previousEdge == NULL; nei++) {
			if (_edges[nei].first == _edges[i].second->previousEdge)
				brep->_edges[i].second->previousEdge = brep->_edges[nei].first;
			else if (_edges[nei].second == _edges[i].second->previousEdge)
				brep->_edges[i].second->previousEdge = brep->_edges[nei].second;
		}
		if (brep->_edges[i].first->nextEdge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
		if (brep->_edges[i].first->previousEdge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
		if (brep->_edges[i].second->nextEdge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
		if (brep->_edges[i].second->previousEdge == NULL)
			RW_THROW("Make sure BREP is valid before scaling it.");
	}
}

void BREP::addBREPEdge(const std::size_t curveIndex, const std::size_t vertex1, const std::size_t vertex2) {
	_edges.push_back(std::make_pair(new HalfEdge(curveIndex),new HalfEdge(curveIndex)));
	_edges.back().first->previousVertex = _vertices[vertex1];
	_edges.back().first->nextVertex = _vertices[vertex2];
	_edges.back().first->oppositeEdge = _edges.back().second;
	_edges.back().first->reversed = false;
	_edges.back().second->previousVertex = _vertices[vertex2];
	_edges.back().second->nextVertex = _vertices[vertex1];
	_edges.back().second->oppositeEdge = _edges.back().first;
	_edges.back().second->reversed = true;
	_edges.back().first->nextEdge = _edges.back().second;
	_edges.back().first->previousEdge = _edges.back().second;
	_edges.back().second->nextEdge = _edges.back().first;
	_edges.back().second->previousEdge = _edges.back().first;

	if (_vertices[vertex1]->nextEdge == NULL) {
		_vertices[vertex1]->nextEdge = _edges.back().first;
	} else {
		HalfEdge* const vPrev = freeHalfEdge(*_vertices[vertex1]);
		HalfEdge* const vNext = vPrev->nextEdge;
		if (vPrev == vNext) {
			// Circle...
		}
		vPrev->nextEdge = _edges.back().first;
		_edges.back().first->previousEdge = vPrev;
		_edges.back().second->nextEdge = vNext;
		vNext->previousEdge = _edges.back().second;
	}
	if (vertex1 == vertex2)
		return;
	if (_vertices[vertex2]->nextEdge == NULL) {
		_vertices[vertex2]->nextEdge = _edges.back().second;
	} else {
		HalfEdge* const vPrev = freeHalfEdge(*_vertices[vertex2]);
		HalfEdge* const vNext = vPrev->nextEdge;
		if (vPrev == vNext) {
			// Circle...
		}
		vPrev->nextEdge = _edges.back().second;
		_edges.back().second->previousEdge = vPrev;
		_edges.back().first->nextEdge = vNext;
		vNext->previousEdge = _edges.back().first;
	}
}

void BREP::setBREPFace(std::size_t surfaceIndex, std::size_t loop) {
	_faces[loop]->surfaceIndex = surfaceIndex;
	_faces[loop]->surfaceSet = true;
	HalfEdge* edge = _faces[loop]->edge->nextEdge;
	while (edge != _faces[loop]->edge) {
		edge->face = _faces[loop];
		edge = edge->nextEdge;
	}
}

void BREP::print() {
	std::cout << "Vertices: " << _vertices.size() << std::endl;
	for (std::size_t i = 0; i < _vertices.size(); i++) {
		std::cout << " ";
		if (_vertices[i]->nextEdge == NULL) {
			std::cout << " Edge NULL";
		} else {
			bool found = false;
			for (std::size_t k = 0; k < _edges.size() && !found; k++) {
				if (_vertices[i]->nextEdge == _edges[k].first) {
					std::cout << " Edge fwd " << k;
					found = true;
				} else if (_vertices[i]->nextEdge == _edges[k].second) {
					std::cout << " Edge bwd " << k;
					found = true;
				}
			}
			if (!found)
				std::cout << "Edge not found " << _vertices[i]->nextEdge;
		}
		std::cout << "  Point " << _vertices[i]->point << std::endl;
	}
	std::cout << "Edges: " << _edges.size() << std::endl;
	for (std::size_t i = 0; i < _edges.size(); i++) {
		std::cout << " ";

		if (_edges[i].first->previousVertex == NULL) {
			std::cout << " Prev Vertex NULL";
		} else {
			bool found = false;
			for (std::size_t k = 0; k < _vertices.size() && !found; k++) {
				if (_edges[i].first->previousVertex == _vertices[k]) {
					std::cout << " Prev Vertex " << k;
					found = true;
				}
			}
			if (!found)
				std::cout << "Vertex not found " << _edges[i].first->previousVertex;
		}

		if (_edges[i].first->nextVertex == NULL) {
			std::cout << " Next Vertex NULL";
		} else {
			bool found = false;
			for (std::size_t k = 0; k < _vertices.size() && !found; k++) {
				if (_edges[i].first->nextVertex == _vertices[k]) {
					std::cout << " Next Vertex " << k;
					found = true;
				}
			}
			if (!found)
				std::cout << "Vertex not found " << _edges[i].first->previousVertex;
		}

		if (_edges[i].first->nextEdge == NULL) {
			std::cout << " Next Edge NULL";
		} else {
			bool found = false;
			for (std::size_t k = 0; k < _edges.size() && !found; k++) {
				if (_edges[i].first->nextEdge == _edges[k].first) {
					std::cout << " Next Edge fwd " << k;
					found = true;
				} else if (_edges[i].first->nextEdge == _edges[k].second) {
					std::cout << " Next Edge bwd " << k;
					found = true;
				}
			}
			if (!found)
				std::cout << "Edge not found " << _edges[i].first->nextEdge;
		}

		/*
		const QuadraticCurve& curve = getCurve(_edges[i].first->curveIndex);
		std::cout << " " << curve.c() << " " << curve.u() << " " << curve.v() << " " << curve.type();
		if (curve.hasLimits())
			std::cout << " " << curve.limits().first << " " << curve.limits().second;
		*/

		std::cout << std::endl;
	}
	std::set<HalfEdge*> edges;
	for (std::size_t i = 0; i < _edges.size(); i++) {
		edges.insert(_edges[i].first);
		edges.insert(_edges[i].second);
	}
	std::cout << "Connected halfedges:" << std::endl;
	while(edges.size() > 0) {
		HalfEdge* const begin = *edges.begin();
		HalfEdge* current = begin;
		std::cout << " ";
		do {
			for (std::size_t k = 0; k < _edges.size(); k++) {
				if (current == _edges[k].first) {
					std::cout << " " << k+1;
					break;
				} else if (current == _edges[k].second) {
					std::cout << " -" << k+1;
					break;
				}
			}
			edges.erase(current);
			current = current->nextEdge;
			bool currentInSet = false;
			for (std::set<HalfEdge*>::const_iterator eIt = edges.begin(); eIt != edges.end(); eIt++) {
				if (*eIt == current) {
					currentInSet = true;
					break;
				}
			}
			if (!currentInSet && current != begin) {
				std::cout << "Repeated edge encountered! Current: ";
				for (std::size_t k = 0; k < _edges.size(); k++) {
					if (current == _edges[k].first) {
						std::cout << " " << k+1;
						break;
					} else if (current == _edges[k].second) {
						std::cout << " -" << k+1;
						break;
					}
				}
				break;
			}
		} while (current != begin && edges.size() > 0);
		std::cout << std::endl;
	}
	for (std::size_t i = 0; i < _edges.size(); i++) {
		edges.insert(_edges[i].first);
		edges.insert(_edges[i].second);
	}
	std::cout << "Connected halfedges backward:" << std::endl;
	while(edges.size() > 0) {
		HalfEdge* const begin = *edges.begin();
		HalfEdge* current = begin;
		std::cout << " ";
		do {
			for (std::size_t k = 0; k < _edges.size(); k++) {
				if (current == _edges[k].first) {
					std::cout << " " << k+1;
					break;
				} else if (current == _edges[k].second) {
					std::cout << " -" << k+1;
					break;
				}
			}
			edges.erase(current);
			current = current->previousEdge;
			bool currentInSet = false;
			for (std::set<HalfEdge*>::const_iterator eIt = edges.begin(); eIt != edges.end(); eIt++) {
				if (*eIt == current) {
					currentInSet = true;
					break;
				}
			}
			if (!currentInSet && current != begin) {
				std::cout << "Repeated edge encountered! Current: ";
				for (std::size_t k = 0; k < _edges.size(); k++) {
					if (current == _edges[k].first) {
						std::cout << " " << k+1;
						break;
					} else if (current == _edges[k].second) {
						std::cout << " -" << k+1;
						break;
					}
				}
				break;
			}
		} while (current != begin && edges.size() > 0);
		std::cout << std::endl;
	}
	std::set<const HalfEdge*> faceEdges;
	std::cout << "Faces: " << _faces.size() << std::endl;
	for (std::size_t i = 0; i < _faces.size(); i++) {
		const HalfEdge* const begin = _faces[i]->edge;
		const HalfEdge* current = begin;
		std::cout << " ";
		do {
			for (std::size_t k = 0; k < _edges.size(); k++) {
				if (current == _edges[k].first) {
					std::cout << " " << k+1;
					break;
				} else if (current == _edges[k].second) {
					std::cout << " -" << k+1;
					break;
				}
			}
			current = current->nextEdge;
			bool currentInSet = false;
			for (std::set<const HalfEdge*>::const_iterator eIt = faceEdges.begin(); eIt != faceEdges.end(); eIt++) {
				if (*eIt == current) {
					currentInSet = true;
					break;
				}
			}
			if (currentInSet && current != begin) {
				std::cout << "Repeated edge encountered! Current: ";
				for (std::size_t k = 0; k < _edges.size(); k++) {
					if (current == _edges[k].first) {
						std::cout << " " << k+1;
						break;
					} else if (current == _edges[k].second) {
						std::cout << " -" << k+1;
						break;
					}
				}
				std::cout << " Prev: ";
				for (std::size_t k = 0; k < _edges.size(); k++) {
					if (current->previousEdge == _edges[k].first) {
						std::cout << " " << k+1;
						break;
					} else if (current->previousEdge == _edges[k].second) {
						std::cout << " -" << k+1;
						break;
					}
				}
				std::cout << " Prev prev: ";
				for (std::size_t k = 0; k < _edges.size(); k++) {
					if (current->previousEdge->previousEdge == _edges[k].first) {
						std::cout << " " << k+1;
						break;
					} else if (current->previousEdge->previousEdge == _edges[k].second) {
						std::cout << " -" << k+1;
						break;
					}
				}
				break;
			}
			faceEdges.insert(current);
		} while (current != begin);

		/*
		if (_faces[i]->surfaceSet) {
			const QuadraticSurface& surface = getSurface(_faces[i]->surfaceIndex);
			if (surface.diagonalized())
				std::cout << " A: " << surface.A()(0,0) << " " << surface.A()(1,1) << " " << surface.A()(2,2);
			else
				std::cout << " A: " << surface.A()(0,0) << " " << surface.A()(0,1) << " " << surface.A()(0,2) << " " << surface.A()(1,0) << " " << surface.A()(1,1) << " " << surface.A()(1,2) << " " << surface.A()(2,0) << " " << surface.A()(2,1) << " " << surface.A()(2,2);
			std::cout << " a: " << surface.a().transpose() << " u: " << surface.u();
		}
		*/

		std::cout << std::endl;
	}
}
