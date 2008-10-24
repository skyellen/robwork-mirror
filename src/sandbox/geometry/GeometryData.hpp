/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef GEOMETRYDATA_HPP_
#define GEOMETRYDATA_HPP_

#include <rw/common/Ptr.hpp>

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

#endif /* GEOMETRYDATA_HPP_ */
