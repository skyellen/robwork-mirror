/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "QuadraticBREP.hpp"
#include "QuadraticCurve.hpp"
#include "QuadraticFace.hpp"
#include "QuadraticSurface.hpp"
#include "PlainQuadraticShell.hpp"

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

namespace {
// Shell Proxy class
class BREPQuadraticShell: public QuadraticShell {
public:
	BREPQuadraticShell(const QuadraticBREP* brep, double resolution): QuadraticShell(), _brep(brep) {
		_resolution = resolution;
	}
	~BREPQuadraticShell() {}
	bool isConvex() { return _brep->isConvex(); }
	std::size_t size() const { return _brep->faceCount(); }
	QuadraticFace::CPtr getFace(std::size_t idx) const {
		const QuadraticFace::Ptr bface = ownedPtr(new QuadraticFace(_brep->getSurface(idx).clone(), _brep->getVertices(idx)));
		const std::vector<QuadraticCurve::Ptr> curves = _brep->getCurves(idx);
		for (std::size_t i = 0; i < curves.size(); i++) {
			bface->setCurve(i,curves[i]);
		}
		bface->setMeshResolution(_resolution);
		return bface;
	}
	void getFace(std::size_t idx, QuadraticFace& dst) const {
		dst.setMeshResolution(_resolution);
		dst.setSurface(_brep->getSurface(idx));
		dst.setVertices(_brep->getVertices(idx));
		const std::vector<QuadraticCurve::Ptr> curves = _brep->getCurves(idx);
		for (std::size_t i = 0; i < curves.size(); i++) {
			dst.setCurve(i,curves[i]);
		}
	}

private:
	Face::CPtr doGetFace(std::size_t idx) const {
		return getFace(idx);
	}

	const QuadraticBREP* const _brep;
};
}

QuadraticBREP::QuadraticBREP(): BREP() {
	setMeshResolution(10);
}

QuadraticBREP::~QuadraticBREP() {
}

GeometryData::GeometryType QuadraticBREP::getType() const {
	return GeometryData::Quadratic;
}

const QuadraticSurface& QuadraticBREP::getSurface(std::size_t idx) const {
	return *_surfaces[idx];
}

const QuadraticCurve& QuadraticBREP::getCurve(std::size_t idx) const {
	return *_curves[idx];
}

std::vector<QuadraticCurve::Ptr> QuadraticBREP::getCurves(std::size_t loopIdx) const {
	std::vector<QuadraticCurve::Ptr> curves;
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

QuadraticShell::CPtr QuadraticBREP::shellProxy() const {
	return ownedPtr(new BREPQuadraticShell(this,_resolution));
}

Shell::CPtr QuadraticBREP::doShellProxyBREP() const {
	return ownedPtr(new BREPQuadraticShell(this,_resolution));
}

PlainQuadraticShell::Ptr QuadraticBREP::shell() const {
	PlainQuadraticShell::Ptr shell = ownedPtr(new PlainQuadraticShell());
	for (std::size_t i = 0; i < faceCount(); i++) {
		const QuadraticFace::Ptr bface = ownedPtr(new QuadraticFace(getSurface(i).clone(), getVertices(i)));
		const std::vector<QuadraticCurve::Ptr> curves = getCurves(i);
		for (std::size_t i = 0; i < curves.size(); i++) {
			bface->setCurve(i,curves[i]);
		}
		shell->add(bface);
	}
	return shell;
}

void QuadraticBREP::scale(double factor) {
	for (std::size_t i = 0; i < _vertices.size(); i++) {
		_vertices[i]->point *= factor;
	}
	for (std::size_t i = 0; i < _surfaces.size(); i++) {
		_surfaces[i] = _surfaces[i]->scale(factor);
	}
	for (std::size_t i = 0; i < _curves.size(); i++) {
		_curves[i] = _curves[i]->scale(factor);
	}
}

QuadraticBREP::Ptr QuadraticBREP::clone() const {
	const QuadraticBREP::Ptr brep = ownedPtr(new QuadraticBREP());
	copyTopologyTo(brep);
	brep->_surfaces.resize(_surfaces.size());
	brep->_curves.resize(_curves.size());
	for (std::size_t i = 0; i < _surfaces.size(); i++) {
		brep->_surfaces[i] = _surfaces[i]->clone();
	}
	for (std::size_t i = 0; i < _curves.size(); i++) {
		brep->_curves[i] = _curves[i]->clone();
	}
	return brep;
}

void QuadraticBREP::addEdge(const QuadraticCurve& curve, std::size_t v1, std::size_t v2) {
	_curves.push_back(curve.clone());
	addBREPEdge(_curves.size()-1,v1,v2);
}

void QuadraticBREP::setFace(const QuadraticSurface& surface, std::size_t loop) {
	std::size_t surfaceIndex;
	if (hasSurfaceSet(loop)) {
		surfaceIndex = getSurfaceIndex(loop);
		_surfaces[surfaceIndex] = surface.clone();
	} else {
		_surfaces.push_back(surface.clone());
		surfaceIndex = _surfaces.size()-1;
	}
	setBREPFace(surfaceIndex, loop);
}

class QuadraticBREP::CommonQuadraticCurveSetImpl: public CommonQuadraticCurveSet {
public:
	CommonQuadraticCurveSetImpl(const QuadraticBREP* brep, const std::vector<const BREP::HalfEdge*>& edges): CommonQuadraticCurveSet(), _brep(brep), _edges(edges) {}
	virtual ~CommonQuadraticCurveSetImpl() {}
	virtual std::size_t size() const { return _edges.size(); }
	virtual const QuadraticCurve& curve(std::size_t index) const { return _brep->getCurve(_edges[index]->curveIndex); }
	virtual const QuadraticSurface& surfaceLeft(std::size_t index) const {
		return _brep->getSurface(_edges[index]->face->surfaceIndex);
	}
	virtual const QuadraticSurface& surfaceRight(std::size_t index) const {
		return _brep->getSurface(_edges[index]->oppositeEdge->face->surfaceIndex);
	}
private:
	const QuadraticBREP* const _brep;
	const std::vector<const BREP::HalfEdge*> _edges;
};

QuadraticBREP::CommonQuadraticCurveSet::CPtr QuadraticBREP::getCommonCurves(const std::set<std::size_t>& faces) const {
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
	return ownedPtr(new CommonQuadraticCurveSetImpl(this, edges));
}
