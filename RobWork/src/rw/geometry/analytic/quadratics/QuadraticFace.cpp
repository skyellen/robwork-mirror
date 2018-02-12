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

#include "QuadraticFace.hpp"
#include "QuadraticCurve.hpp"
#include "QuadraticSurface.hpp"

using rw::common::ownedPtr;
using namespace rw::geometry;
using namespace rw::math;

QuadraticFace::QuadraticFace()
{
}

QuadraticFace::QuadraticFace(rw::common::Ptr<const QuadraticSurface> surface, const std::vector<Vector3D<> >& vertices):
	_surface(surface),
	_curves(vertices.size()),
	_vertices(vertices)
{
}

QuadraticFace::~QuadraticFace() {
}

const QuadraticSurface& QuadraticFace::surface() const {
	return *_surface;
}

const QuadraticCurve& QuadraticFace::getCurve(std::size_t i) const {
	return *_curves[i];
}

void QuadraticFace::transform(const Vector3D<>& P) {
	_surface = _surface->transform(P);
	for (std::size_t i = 0; i < _vertices.size(); i++) {
		_vertices[i] += P;
	}
	for (std::size_t i = 0; i < _curves.size(); i++) {
		_curves[i] = _curves[i]->transform(P);
	}
}

void QuadraticFace::transform(const Transform3D<>& T) {
	_surface = _surface->transform(T);
	for (std::size_t i = 0; i < _vertices.size(); i++) {
		_vertices[i] = T*_vertices[i];
	}
	for (std::size_t i = 0; i < _curves.size(); i++) {
		_curves[i] = _curves[i]->transform(T);
	}
}

void QuadraticFace::setSurface(const QuadraticSurface& surface) {
	_surface = surface.clone();
}

void QuadraticFace::setCurve(std::size_t vertex, QuadraticCurve::CPtr curve) {
	if (vertex >= _vertices.size()) {
		_vertices.resize(vertex+1);
		_curves.resize(vertex+1);
	}
	_curves[vertex] = curve;
}

void QuadraticFace::setCurves(const std::vector<QuadraticCurve::CPtr>& curves) {
	_curves = curves;
	_vertices.resize(curves.size());
}

void QuadraticFace::setVertex(std::size_t index, const Vector3D<>& vertex) {
	if (index >= _vertices.size()) {
		_vertices.resize(index+1);
		_curves.resize(index+1);
	}
	_vertices[index] = vertex;
}

void QuadraticFace::setVertices(const std::vector<Vector3D<> >& vertices) {
	_vertices = vertices;
	_curves.resize(vertices.size());
}
