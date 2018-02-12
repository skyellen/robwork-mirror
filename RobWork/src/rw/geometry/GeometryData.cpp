/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "GeometryData.hpp"

#include <string>

using namespace rw::geometry;

std::string GeometryData::toString(GeometryData::GeometryType type){
	switch(type){
    case(PointCloud): return "PointCloud";
	case(PlainTriMesh): return "PlainTriMesh";
	case(IdxTriMesh): return "IdxTriMesh";
	case(SpherePrim): return "SpherePrim";
	case(BoxPrim): return "BoxPrim";
	case(OBBPrim): return "OBBPrim";
	case(AABBPrim): return "AABBPrim";
	case(LinePrim): return "LinePrim";
	case(PointPrim): return "PointPrim";
	case(PyramidPrim): return "PyramidPrim";
	case(ConePrim): return "ConePrim";
	case(TrianglePrim): return "TrianglePrim";
	case(CylinderPrim): return "CylinderPrim";
	case(TubePrim): return "TubePrim";
	case(PlanePrim): return "PlanePrim";
	case(RayPrim): return "RayPrim";
	case(Quadratic): return "Quadratic";
	case(UserType): return "UserType";
	default:
		return "Not Supported!";
	}
	return "Not Supported!";
}
