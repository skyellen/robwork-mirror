/*
 * SurfacePoseSampler.hpp
 *
 *  Created on: Aug 23, 2011
 *      Author: jimali
 */

#ifndef SURFACEPOSESAMPLER_HPP_
#define SURFACEPOSESAMPLER_HPP_

#include "PoseSampler.hpp"
#include <rw/geometry/Geometry.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/geometry/TriMesh.hpp>
/**
 * @brief random sampling of points and orientations close to the surface of a geometry.
 *
 * A point \b p on the surface is randomly choosen.<br>
 * A rotation \b rot is randomly generated.<br>
 * A random distance \b d in the interval [minD, maxD] is generated<br>
 * The position \b pos is calculated as \f$ pos = p - rot*(0,0,d)^T \f$ <br>
 * The random pose \b X is thus \f$ X = (pos, rot) \f$
 * Optionally a random rotation of \b X can be generated such that z-axis of rot is
 * not allways pointing toward the surface.
 */
class SurfacePoseSampler: public PoseSampler {
public:
    typedef rw::common::Ptr<SurfacePoseSampler> Ptr;

    SurfacePoseSampler(rw::geometry::Geometry::Ptr geom):
        _minD(0.02),
        _maxD(0.02),
        _genRandomRotation(true),
        _filterByDirection(false),
        _genRandomPostion(true)
    {
        init(geom);
    }

    SurfacePoseSampler(const std::vector<rw::geometry::Geometry::Ptr>& geoms):
        _minD(0.02),
        _maxD(0.02),
        _genRandomRotation(true),
        _filterByDirection(false),
        _genRandomPostion(true)
    {
        init(geoms[0]);
    }

    virtual ~SurfacePoseSampler(){}

    void init(rw::geometry::Geometry::Ptr geom){
        _mesh = geom->getGeometryData()->getTriMesh();
        _surfaceArea.resize(_mesh->size());
        _sAreaSum = 0;
        // run through mesh and create the search
        for(size_t i=0; i<_mesh->size(); i++){
            rw::geometry::Triangle<> tri = _mesh->getTriangle(i);
            // calculate triangle area
            _sAreaSum += tri.calcArea();
            _surfaceArea[i] = _sAreaSum;
        }
    }


    rw::math::Transform3D<> sample(){
        using namespace rw::math;
        Transform3D<> target;
        do{
            double rnum = rw::math::Math::ran(0.0, _sAreaSum);
            int triIds = binSearchRec(rnum, 0, _mesh->size()-1);
            rw::geometry::Triangle<> tri = _mesh->getTriangle(triIds);

            // random sample the triangle
            double b0 = Math::ran();
            double b1 = ( 1.0f - b0 ) * Math::ran();
            double b2 = 1 - b0 - b1;

            Vector3D<> position = tri[0] * b0 + tri[1] * b1 + tri[2] * b2;

            // and sample the orientation
            //EAA<> eaa(Vector3D<>::z(), -tri.calcFaceNormal());
            if(_genRandomPostion){
                target = Transform3D<>( position, Math::ranRotation3D<double>());
                target.P() -= (target.R()*Vector3D<>::z())*Math::ran(_minD,_maxD);
            } else {
                target.P() = position;
                // align z-axis of rotation with triangle normal
                EAA<> eaa( Vector3D<>::z(), tri.calcFaceNormal() );
                target.R() = eaa.toRotation3D();
            }

            if(_genRandomRotation){
                target.R() = Math::ranRotation3D<double>();
            }
            if(!_filterByDirection)
                return target;
        } while ( dot(_direction,target.R()*Vector3D<>::z())>0);
        return target;
    }

    void setBoundsD(double minD, double maxD){
        _minD = minD;
        _maxD = maxD;
    }

    void setRandomRotationEnabled(bool enabled){
        _genRandomRotation = enabled;
    }

    void setRandomPositionEnabled(bool enabled){
        _genRandomPostion = enabled;
    }

    void setZAxisDirectionEnabled(bool enabled) { _filterByDirection = enabled; }

    void setZAxisDirection(const rw::math::Vector3D<>& dir){_direction = dir;}


private:
    int binSearchRec(const double value, size_t start, size_t end){
        if(start==end)
            return (int)start;
        // choose a int between start and end
        size_t split = (end-start)/2+start;
        if(value<_surfaceArea[split])
            return binSearchRec(value, start, split);
        else
            return binSearchRec(value, split+1, end);
    }

private:
    double _sAreaSum, _minD, _maxD;
    bool _genRandomRotation, _filterByDirection, _genRandomPostion;
    std::vector<double> _surfaceArea;
    rw::geometry::TriMesh::Ptr _mesh;
    rw::math::Vector3D<> _direction;
};




#endif /* SURFACEPOSESAMPLER_HPP_ */
