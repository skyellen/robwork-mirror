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

#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>

#include "GeometryData.hpp"

namespace rw { namespace geometry {
namespace sandbox {

class Geometry {

public:

    Geometry(GeometryDataPtr data):
        _data(data),
        _transform(rw::math::Transform3D<>::identity() )
    {
    };

    Geometry(GeometryDataPtr data,
             const rw::math::Transform3D<>& t3d):

        _data(data),
        _transform(t3d)
    {
    };

    virtual ~Geometry(){};

    void setTransform(const rw::math::Transform3D<>& t3d){_transform = t3d;};
    const rw::math::Transform3D<>& getTransform(){return _transform;};

    GeometryDataPtr getGeometryData(){return _data;};
    void setGeometryData(GeometryDataPtr data){_data = data;};

    GeometryData* getBV(){return _bv;};
    void setBV(GeometryData* bv){_bv = bv;};

private:

    GeometryDataPtr _data;
    GeometryData *_bv;
    rw::math::Transform3D<> _transform;

};
}
}}

#endif /* GEOMETRY_HPP_ */
