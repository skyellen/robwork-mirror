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

#include "PolygonUtil.hpp"

#include <stack>

using namespace rw::math;
using namespace rw::geometry;

namespace {
class Polygon2D {
public:
	typedef std::list<std::pair<std::size_t,std::size_t> > CutList;

private:
	struct StackEntry {
		StackEntry(std::size_t prev, const CutList& polygons): prev(prev), polygons(polygons) {}
		const std::size_t prev;
		CutList polygons;
	};

	struct DecomposeData {
		std::stack<StackEntry> stack;
		CutList best;
	};

public:
	Polygon2D(const Polygon<Vector2D<> >& polygon): _p(polygon.size()), reversed(false) {
		for (std::size_t i = 0; i < polygon.size(); i++) {
			_p[i] = polygon[i];
		}
		reversed = PolygonUtil::area(polygon) > 0;
		if (reversed) {
			std::reverse(_p.begin(),_p.end());
		}
	}
	~Polygon2D() {}

	std::vector<std::vector<std::size_t> > decompose() {
		const CutList cuts = decompose(0,_p.size()-1);
		std::vector<bool> remains(_p.size(),true);
		std::vector<std::vector<std::size_t> > res(cuts.size());
		std::size_t ci = 0;
		for (CutList::const_iterator cit = cuts.begin(); cit != cuts.end(); cit++) {
			const std::size_t i = cit->first;
			const std::size_t j = cit->second;
			res[ci].push_back(i);
			for (std::size_t vi = (i+1)%_p.size(); vi != j; vi = (vi+1)%_p.size()) {
				if (remains[vi]) {
					res[ci].push_back(vi);
				}
				remains[vi] = false;
			}
			res[ci].push_back(j);
			ci++;
		}
		if (reversed) {
			for (std::size_t i = 0; i < res.size(); i++) {
				for (std::size_t j = 0; j < res[i].size(); j++) {
					res[i][j] = _p.size()-1-res[i][j];
				}
				std::reverse(res[i].begin(),res[i].end());
			}
		}
		return res;
	}

	CutList decompose(std::size_t i, std::size_t j) {
		if ((i+1)%_p.size() == j) {
			return CutList();
		}
		if (_optimal[i][j].size() != 0) {
			return _optimal[i][j];
		}
		_decRunning[i][j] = true;
		std::vector<std::size_t> visible(1,i);
		for (std::size_t k = (i+1)%_p.size(); k != (j+1)%_p.size(); k = (k+1)%_p.size()) {
			if (_visible[i][k] && _visible[k][j])
				visible.push_back(k);
		}
		std::vector<DecomposeData> stack(_p.size());
		for (std::size_t ki = 1; ki < visible.size(); ki++) {
			load(ki,visible,stack);
		}
		_optimal[i][j] = best(j,i,stack);
		_optimal[i][j].push_back(std::make_pair(i,j));
		_decRunning[i][j] = false;
		return _optimal[i][j];
	}

	void load(std::size_t currentI, const std::vector<std::size_t>& vertexList, std::vector<DecomposeData>& stack) {
		for (std::size_t i = 0; i < currentI; i++) {
			const std::size_t prev = vertexList[currentI-1-i];
			const std::size_t current = vertexList[currentI];
			if ((_valid[prev][current] && !_decRunning[prev][current]) || (_visible[prev][current] && !stack[prev].stack.empty())) {
				CutList dec = decompose(prev,current);
				const CutList b = best(prev,current,stack);
				dec.insert(dec.end(), b.begin(), b.end());
				stack[current].stack.push(StackEntry(prev,dec));
				stack[current].best = stack[current].stack.top().polygons;
			}
		}
	}

	CutList best(std::size_t pivot, std::size_t extension, std::vector<DecomposeData>& stack) {
		CutList res = stack[pivot].best;
		while (stack[pivot].stack.size() > 0) {
			const StackEntry& old = stack[pivot].stack.top();

			const Vector3D<> v1(_p[pivot][0]-_p[old.prev][0],_p[pivot][1]-_p[old.prev][1],0);
			const Vector3D<> v2(_p[extension][0]-_p[pivot][0],_p[extension][1]-_p[pivot][1],0);
			if (cross(v1,v2)[2] > 0) {
				// concave
				return res;
			} else if (old.polygons.size() < res.size()) {
				res = old.polygons;
			}
			stack[pivot].best = old.polygons;
			stack[pivot].stack.pop();
		}
		return res;
	}

	void preprocess() {
		_visible.resize(_p.size(),std::vector<bool>(_p.size(),false));
		_valid.resize(_p.size(),std::vector<bool>(_p.size(),false));
		_decRunning.resize(_p.size(),std::vector<bool>(_p.size(),false));
		_optimal.resize(_p.size(),std::vector<CutList>(_p.size()));

		for (std::size_t i = 0; i < _p.size(); i++) {
			_visible[i][i] = true;
			_valid[i][i] = inwardCusp(i);
			for (std::size_t j = i+1; j < _p.size(); j++) {
				if (visibleTo(i,j)) {
					_visible[i][j] = true;
					_visible[j][i] = _visible[i][j];
					_valid[i][j] = inwardCusp(i) || inwardCusp(j);
					_valid[j][i] = _valid[i][j];
				}
			}
		}
	}

	bool visibleTo(std::size_t vertex, std::size_t visibleTo) const {
		// Test for intersections of line segments
		for (std::size_t i = 0; i < _p.size(); i++) {
			const std::size_t j = (i+1)%_p.size();
			if (i == vertex || j == vertex || j == visibleTo || i == visibleTo)
				continue;
			const double den = (_p[i][0]-_p[j][0])*(_p[vertex][1]-_p[visibleTo][1])-(_p[i][1]-_p[j][1])*(_p[vertex][0]-_p[visibleTo][0]);
			if (den == 0)
				continue;
			Vector2D<> cr;
			cr[0] = (_p[i][0]*_p[j][1]-_p[i][1]*_p[j][0])*(_p[vertex][0]-_p[visibleTo][0])-(_p[i][0]-_p[j][0])*(_p[vertex][0]*_p[visibleTo][1]-_p[vertex][1]*_p[visibleTo][0]);
			cr[1] = (_p[i][0]*_p[j][1]-_p[i][1]*_p[j][0])*(_p[vertex][1]-_p[visibleTo][1])-(_p[i][1]-_p[j][1])*(_p[vertex][0]*_p[visibleTo][1]-_p[vertex][1]*_p[visibleTo][0]);
			cr /= den;
			const Vector2D<> dIJ = _p[j]-_p[i];
			const double lenIJsq = std::pow(dIJ.norm2(),2);
			const Vector2D<> dC = _p[vertex]-_p[visibleTo];
			const double lenCsq = std::pow(dC.norm2(),2);
			if (dot(cr-_p[i],dIJ) < lenIJsq && dot(_p[j]-cr,dIJ) < lenIJsq && dot(cr-_p[visibleTo],dC) < lenCsq && dot(_p[vertex]-cr,dC) < lenCsq)
				return false;
		}
		// Test if edge is inside or outside polygon
		const std::size_t prev = (vertex==0)?_p.size()-1:vertex-1;
		const std::size_t next = (vertex+1)%_p.size();
		if (visibleTo != next && visibleTo != prev) {
			const Vector3D<> v = normalize(Vector3D<>(_p[visibleTo][0]-_p[vertex][0],_p[visibleTo][1]-_p[vertex][1],0));
			const Vector3D<> vprev = normalize(Vector3D<>(_p[vertex][0]-_p[prev][0],_p[vertex][1]-_p[prev][1],0));
			const Vector3D<> vnext = normalize(Vector3D<>(_p[next][0]-_p[vertex][0],_p[next][1]-_p[vertex][1],0));
			if (std::atan2(cross(vprev,v)[2],dot(vprev,v)) > std::atan2(cross(vprev,vnext)[2],dot(vprev,vnext)))
				return false;
		}

		return true;
	}

	bool inwardCusp(std::size_t vertex) const {
		const std::size_t prev = (vertex==0)?_p.size()-1:vertex-1;
		const std::size_t next = (vertex+1)%_p.size();
		const Vector3D<> v1(_p[vertex][0]-_p[prev][0],_p[vertex][1]-_p[prev][1],0);
		const Vector3D<> v2(_p[next][0]-_p[vertex][0],_p[next][1]-_p[vertex][1],0);
		return cross(v1,v2)[2] > 0;
	}

private:
	std::vector<Vector2D<> > _p;
	std::vector<std::vector<bool> > _visible;
	std::vector<std::vector<bool> > _valid;
	std::vector<std::vector<bool> > _decRunning;
	std::vector<std::vector<CutList> > _optimal;
	bool reversed;
};
}

PolygonUtil::PolygonUtil() {
}

PolygonUtil::~PolygonUtil() {
}

std::vector<std::vector<std::size_t> > PolygonUtil::convexDecompositionIndexed(const Polygon<Vector2D<> >& polygon) {
	Polygon2D p(polygon);
	p.preprocess();
	return p.decompose();
}

std::vector<Polygon<Vector2D<> > > PolygonUtil::convexDecomposition(const Polygon<Vector2D<> >& polygon) {
	const std::vector<std::vector<std::size_t> > polygons = convexDecompositionIndexed(polygon);

	std::vector<Polygon<Vector2D<> > > res(polygons.size());
	std::size_t i = 0;
	for (std::vector<std::vector<std::size_t> >::const_iterator it = polygons.begin(); it != polygons.end(); it++) {
		const std::vector<std::size_t>& subPoly = *it;
		for (std::size_t k = 0; k < subPoly.size(); k++) {
			res[i].addVertex(polygon[subPoly[k]]);
		}
		i++;
	}
	return res;
}

bool PolygonUtil::isInsideConvex(const Vector2D<>& point, const Polygon<Vector2D<> >& polygon) {
	unsigned int pos = 0;
	unsigned int neg = 0;

	for (std::size_t i = 0; i < polygon.size(); i++) {
		if (polygon[i] == point)
			return false;

		const std::size_t iNext = (i == polygon.size()-1)? 0 : i+1;
		const double d = (point[0]-polygon[i][0])*(polygon[iNext][1]-polygon[i][1]) - (point[1]-polygon[i][1])*(polygon[iNext][0]-polygon[i][0]);

		if (std::fabs(d) < std::numeric_limits<double>::epsilon()*5)
			return false;
		if (d > 0) pos++;
		if (d < 0) neg++;

		//If the sign changes, then point is outside
		if (pos > 0 && neg > 0)
			return false;
	}
	return true;
}

double PolygonUtil::area(const Polygon<Vector2D<> >& polygon) {
	double area = 0;
	for (std::size_t i = 0; i < polygon.size(); i++) {
		const std::size_t n = (i+1)%polygon.size();
		area -= (polygon[n][0]-polygon[i][0])*(polygon[n][1]+polygon[i][1]);
	}
	area /= 2;
	return area;
}
