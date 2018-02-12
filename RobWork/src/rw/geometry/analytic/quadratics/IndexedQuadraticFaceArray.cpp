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

#include "IndexedQuadraticFaceArray.hpp"
#include "QuadraticFace.hpp"

using rw::common::ownedPtr;
using namespace rw::geometry;

struct IndexedQuadraticFaceArray::CenterSort {
	CenterSort() {}

    bool operator()(const IndexedFace& i0, const IndexedFace& i1) {
        return i0.center < i1.center;
    }
};

IndexedQuadraticFaceArray::IndexedQuadraticFaceArray(rw::common::Ptr<const QuadraticShell> quadric):
	_quadric(quadric),
	_faces(quadric->size()),
	_first(0),
	_last(_faces.size())
{
	for (std::size_t i = 0; i < _faces.size(); i++)
		_faces[i].originalID = i;
}

IndexedQuadraticFaceArray::IndexedQuadraticFaceArray(rw::common::Ptr<const QuadraticShell> quadric, const std::vector<IndexedFace>& faces, std::size_t first, std::size_t last):
	_quadric(quadric),
	_faces(faces),
	_first(first),
	_last(last)
{
}

IndexedQuadraticFaceArray::~IndexedQuadraticFaceArray() {
}

bool IndexedQuadraticFaceArray::isConvex() {
	return false;
}

std::size_t IndexedQuadraticFaceArray::size() const {
	return _last-_first;
}

QuadraticFace::CPtr IndexedQuadraticFaceArray::getFace(std::size_t idx) const {
	return _quadric->getFace(_faces[_first+idx].originalID);
}

void IndexedQuadraticFaceArray::getFace(std::size_t idx, QuadraticFace& dst) const {
	dst = *_quadric->getFace(_faces[_first+idx].originalID);
}

IndexedQuadraticFaceArray::IndexedFace IndexedQuadraticFaceArray::getIndexedFace(std::size_t idx) const {
	return _faces[_first+idx];
}

void IndexedQuadraticFaceArray::getIndexedFace(std::size_t idx, IndexedFace& dst) const {
	dst = _faces[_first+idx];
}

void IndexedQuadraticFaceArray::sortAxis(int axis, const rw::math::Transform3D<>& t3d) {
	// first transform the requested splitaxis values
	for(std::size_t i = _first; i < _first+size(); i++) {
		IndexedFace& elem = _faces[i];
		const std::pair<double,double> ext = _quadric->getFace(elem.originalID)->extremums(t3d.R().getCol(axis));
		elem.lower = ext.first+t3d.P()(axis);
		elem.upper = ext.second+t3d.P()(axis);
		elem.center = (ext.first+ext.second)/2;
	}

	std::sort(_faces.begin()+_first, _faces.begin()+_last, CenterSort());
}


IndexedQuadraticFaceArray IndexedQuadraticFaceArray::getSubRange(std::size_t first, std::size_t last) const {
	RW_ASSERT(first<last);
	return IndexedQuadraticFaceArray(_quadric,_faces,_first+first,_first+last);
}
