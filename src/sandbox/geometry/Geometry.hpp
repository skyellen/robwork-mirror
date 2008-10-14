/*
 * Geometry.hpp
 *
 *  Created on: 10-10-2008
 *      Author: jimali
 */

#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/Frame.hpp>

#include "GeometryData.hpp"

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


#endif /* GEOMETRY_HPP_ */
