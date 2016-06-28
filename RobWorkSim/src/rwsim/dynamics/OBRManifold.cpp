#include "OBRManifold.hpp"

#include <rw/math/LinearAlgebra.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rwsim::dynamics;


bool OBRManifold::addPoint(ContactPoint& p){
    //if( !inManifold(p) )
    //    RW_THROW("Not in manifold");

    if(_nrOfContacts==5){
        // if the contact point is too close to any of the current contacts then
        // discard it
        for (int i = 0; i < _nrOfContacts; i++) {
        	if(MetricUtil::dist2(p.p,_points[i].p)<0.000001)
        		return true;
        }
        // if the point is inside the obb then we only need to check if its
        // deeper than the deepest point and if it s replace it
        if( isInsideOBB(p.p) ){
            if(_points[_deepestIdx].penetration<p.penetration)
                _points[_deepestIdx] = p;
            return true;
        }
        // if its outside then we need to remove a point
        fit(p);
        // we remove the least penetrating point that are not on the border of the bounding box
        double minPenetration = 1000;
        int minPenIdx = -1;
        for(int i=0;i<5;i++){
            if(_onBorderMap[i])
                continue;
            if(minPenetration>_points[i].penetration){
                minPenIdx = i;
                minPenetration = _points[i].penetration;
            }
        }
        if(minPenIdx==-1)
            return true; // something went wrong, we keep the new manifold but the old points
        if( _points[minPenIdx].penetration<p.penetration ) {
        	_points[minPenIdx] = p;
        	if( _points[_deepestIdx].penetration<p.penetration )
        		_deepestIdx = minPenIdx;
        }
        return true;
    } else if(_nrOfContacts>1) {
        // only add a point if its on the contact manifold plane
        //double dist = dot(p.p,_normal)+dot(_normal, _points[0].p);
        double dist = dot(p.n,_normal);
        if(acos(dist)>_threshold){
            //std::cout << acos(dist)*Rad2Deg << std::endl;
            return false;
        }

        // if the contact point is too close to any of the current contacts then
        // discard it
        for (int i = 0; i < _nrOfContacts; i++) {
        	if(MetricUtil::dist2(p.p,_points[i].p)<0.000001)
        		return true;
        }

        Vector3D<> nnorm = p.n;
        for(int i=0;i<_nrOfContacts;i++){ nnorm += _points[i].n; }
        nnorm /= _nrOfContacts;
        //_t3d.R() = _t3d.R()*EAA<>(_normal,nnorm).toRotation3D();
        _normal = nnorm;
        Vector3D<> yaxis = normalize(_points[1].p - _points[0].p);
        _t3d.R() = Rotation3D<>(cross(yaxis,_normal),yaxis,_normal);

        fit(p);

        _points[_nrOfContacts] = p;
        if( _points[_deepestIdx].penetration<p.penetration )
            _deepestIdx = _nrOfContacts;
        _nrOfContacts++;

    } else if(_nrOfContacts==1) {
        // only add a point if its on the contact manifold plane
        // here we only check if the angle between the contact normals is not too large
        //double dist = dot(p.p,_normal)+dot(_normal, _points[0].p);
        double dist = dot(p.n,_normal);
        if( acos(dist)>_threshold ){
            //RW_WARN(acos(dist) << ">" << _threshold);
            return false;
        }
        _normal = (p.n + _points[0].n)/2;
        _points[_nrOfContacts] = p;
        _t3d.P() = (p.p + _points[0].p)/2;
        if( _points[_deepestIdx].penetration<p.penetration )
            _deepestIdx = _nrOfContacts;
        _nrOfContacts++;
        // create a Rotation with _normal as z-axis and vector between points as y-axis
        Vector3D<> yaxis = normalize(p.p - _points[0].p);
        _t3d.R() = Rotation3D<>(cross(yaxis,_normal),yaxis,_normal);
    } else {
        _normal = p.n;
        _points[0] = p;
        _t3d.P() = p.p;
        _deepestIdx = 0;
        _nrOfContacts++;
    }
    return true;
}



void OBRManifold::fit(ContactPoint& p){
    // re-fit the bounding box
    Eigen::MatrixXd covar( Eigen::MatrixXd::Zero(2, 2) );

    size_t nrOfContacts = _nrOfContacts-1;

    for(size_t i=0;i<nrOfContacts;i++){
        Vector3D<> proj_p = inverse( _t3d.R() )* _points[i].p;
        _projPoints[i](0) = proj_p(0);
        _projPoints[i](1) = proj_p(1);
    }

    Vector3D<> proj_p = inverse( _t3d.R() ) * p.p;
    _projPoints[nrOfContacts](0) = proj_p(0);
    _projPoints[nrOfContacts](1) = proj_p(1);

    Vector2D<> c(0,0);
    for(size_t i=0;i<nrOfContacts+1;i++){
        c += _projPoints[i];
        covar(0,0) += _projPoints[i](0)*_projPoints[i](0);
        covar(1,1) += _projPoints[i](1)*_projPoints[i](1);
        covar(0,1) += _projPoints[i](0)*_projPoints[i](1);
    }

    const int n = (int)nrOfContacts+1;
    covar(0,0) = covar(0,0)-c(0)*c(0)/n;
    covar(1,1) = covar(1,1)-c(1)*c(1)/n;
    covar(0,1) = covar(0,1)-c(0)*c(1)/n;
    covar(1,0) = covar(0,1);


    typedef std::pair<Eigen::MatrixXd,Eigen::VectorXd > ResultType;
    //std::cout << "COVAR: " << covar << std::endl;
    ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

    // 4.1 create the rotationmatrix from the normalized eigenvectors
    // find max and the second maximal eigenvalue
    size_t maxEigIdx=1, minEigIdx=0;
    double maxEigVal = res.second(maxEigIdx);
    double minEigVal = res.second(minEigIdx);
    if( maxEigVal < minEigVal ){
        std::swap(minEigVal,maxEigVal);
        std::swap(minEigIdx,maxEigIdx);
    }
    // specify x and y axis, x will be the axis with largest spred
    Vector2D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx));
    Vector2D<> minAxis( res.first(0,minEigIdx), res.first(1,minEigIdx));
    //std::cout << maxAxis << minAxis << std::endl;
    const Rotation2D<> rot(normalize(maxAxis),normalize(minAxis));
    const Rotation2D<> rotInv = inverse( rot );

    Vector2D<> max = rotInv * _projPoints[0];
    Vector2D<> min = max;
    size_t x_max = 0, x_min = 0;
    size_t y_max = 0, y_min = 0;

    for(size_t i = 1; i<nrOfContacts+1; i++ ){
        const Vector2D<> prot = rotInv * _projPoints[i];
        if( prot(0)>max(0) ) {
            max(0) = prot(0); x_max = i;
        } else if( prot(0)<min(0) ) {
            min(0) = prot(0); x_min = i;
        }
        if( prot(1)>max(1) ) {
            max(1) = prot(1); y_max = i;
        } else if( prot(1)<min(1) ) {
            min(1) = prot(1); y_min = i;
        }
    }
    for(int i=0;i<4;i++)
        _onBorderMap[i] = false;
    _onBorderMap[x_min] = true;
    _onBorderMap[x_max] = true;
    _onBorderMap[y_min] = true;
    _onBorderMap[y_max] = true;

    // compute halflength of box and its midpoint
    Vector2D<> center = rot*( 0.5*(max+min));
    //Rotation2D<> rotation = rot;
    // transform it back to 3d
    _t3d.P()(0) = center(0);
    _t3d.P()(1) = center(1);
    Vector2D<> hspace = 0.5*(max-min);
    _h(0) = hspace(0);
    _h(1) = hspace(1);
    //_normal = p.n;
    //for(int i=0; i<_nrOfContacts;i++){
    //    _normal += _points[0].n;
    //}
    //_normal /= _nrOfContacts+1;

}
