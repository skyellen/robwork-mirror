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

#include "PlainQuadraticShell.hpp"
#include "QuadraticFace.hpp"

#include <rw/geometry/analytic/GenericFace.hpp>

using namespace rw::geometry;

void PlainQuadraticShell::getFace(std::size_t idx, QuadraticFace& dst) const {
	dst = *_faces[idx];
}

void PlainQuadraticShell::getFace(std::size_t idx, GenericFace& face) const {
	face = *_faces[idx];
}

Face::CPtr PlainQuadraticShell::doGetFace(std::size_t idx) const {
	return _faces[idx];
}
