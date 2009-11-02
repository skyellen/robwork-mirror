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


#ifndef GEOMETRYDATA_HPP_
#define GEOMETRYDATA_HPP_

#include <rw/common/Ptr.hpp>

namespace rw { namespace geometry {
namespace sandbox {

class GeometryData;

typedef rw::common::Ptr<GeometryData> GeometryDataPtr;

class GeometryData {

public:
    typedef enum {PlainTriMesh,
                  IdxTriMesh,
                  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
                  LinePrim, PointPrim, PyramidPrim, ConePrim,
                  TrianglePrim, UserType} GeometryType;

    //typedef enum {Primitive, PlainTriMesh, IdxTriMesh, UserType} GeomClass;
    //typedef enum {} GeomPrim

    //struct {

    //};

    virtual GeometryType getType() = 0;

};
}
}}
#endif /* GEOMETRYDATA_HPP_ */
