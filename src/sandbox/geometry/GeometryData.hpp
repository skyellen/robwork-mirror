/*
 * GeometryData.hpp
 *
 *  Created on: 10-10-2008
 *      Author: jimali
 */

#ifndef GEOMETRYDATA_HPP_
#define GEOMETRYDATA_HPP_

#include <rw/common/Ptr.hpp>

class GeometryData;

typedef rw::common::Ptr<GeometryData> GeometryDataPtr;

class GeometryData {


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

#endif /* GEOMETRYDATA_HPP_ */
