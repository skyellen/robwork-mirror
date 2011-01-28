/*
 * OBRManifold.hpp
 *
 *  Created on: 16/12/2010
 *      Author: jimali
 */

#ifndef OBRMANIFOLD_HPP_
#define OBRMANIFOLD_HPP_

#include "ContactPoint.hpp"
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwsim {
namespace dynamics {

    /**
     * @brief Contact manifold based on Oriented Bounding Boxes
     */
    class OBRManifold {
    public:
        /**
         * @brief
         * @param thres
         * @param sepThres
         * @return
         */
        OBRManifold(double thres = 0.03, double sepThres = 0.01):
            _threshold(thres),
            _sepThreshold(sepThres),
            _deepestIdx(0),
            _nrOfContacts(0)
        {};

        virtual ~OBRManifold(){};

        /**
         * @brief adds and updates the manifold with a new point
         * @param p
         */
        void addPoint(ContactPoint& p){
            using namespace rw::math;
            if( !inManifold(p) )
                RW_THROW("Not in manifold");

            // handle if we have less than 5 points
            if(_nrOfContacts==5){
                // if the point is inside the obb then we only need to check if its
                // deeper than the deepest point and if it s replace it
                if( isInsideOBB(p.p) ){
                    if(_points[_deepestIdx].penetration<p.penetration)
                        _points[_deepestIdx] = p;
                    return;
                }
                // if its outside then we need to remove another point
                fit(p);
                // find the point closest to center of obb
                int minIdx = -1;
                double minDist = MetricUtil::dist1(p.p,_t3d.P());
                for(int i=0;i<5;i++){
                    double dist = MetricUtil::dist1(_points[i].p,_t3d.P());
                    if( dist<minDist && _deepestIdx!=i ){
                        minIdx = i;
                        minDist = dist;
                    }
                }
                if(minIdx<0)
                    return;
                _points[minIdx] = p;
                if( _points[_deepestIdx].penetration>p.penetration )
                    _deepestIdx = minIdx;
            } else if(_nrOfContacts>1) {
                fit(p);
                _normal = p.n;
                _points[_nrOfContacts] = p;
                _nrOfContacts++;
                _deepestIdx = 0;
            } else if(_nrOfContacts==1) {
                _normal = (p.n + _points[0].n)/2;
                _points[_nrOfContacts] = p;
                _t3d.P() = (p.p + _points[0].n)/2;
                if( _points[_deepestIdx].penetration>p.penetration )
                    _deepestIdx = _nrOfContacts;
                _nrOfContacts++;
            } else {
                _normal = p.n;
                _points[0] = p;
                _t3d.P() = p.p;
                _deepestIdx = 0;
                _nrOfContacts++;
            }
        }

        void update(const rw::math::Transform3D<> &aT, const rw::math::Transform3D<> &bT){
            using namespace rw::math;
            // update the position of the contact points
            for(int i=0; i<_nrOfContacts; i++){
                const Vector3D<>& wPa = aT * _points[i].pA;
                const Vector3D<>& wPb = bT * _points[i].pB;
                if( MetricUtil::dist2(wPa,wPb)> _sepThreshold ){
                    //remove point
                    _nrOfContacts--;
                    _points[i] = _points[_nrOfContacts];
                    i--;
                } else {
                    _points[i].p = (wPa+wPb)/2;
                }
            }
        }

        int getNrOfContacts(){ return _nrOfContacts; };

        bool inManifold(ContactPoint& p){
            using namespace rw::math;
            if( _nrOfContacts==0 ){
                return true;
            } else if( _nrOfContacts == 1){
                // check distance to the deepest point
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                //std::cout << "1: Dist too point: " << dist << std::endl;
                if( dist <_threshold ) return true;
                else return false;
            } else if( _nrOfContacts == 2 ){
                // check distance to the line
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                //std::cout << "1: Dist too point: " << dist << std::endl;
                if( dist <_threshold ) return true;
                else return false;
    /*            const Vector3D<> &p1 = p.p;
                const Vector3D<> &x1 = _points[0].p;
                const Vector3D<> &x2 = _points[1].p;
                double dist = MetricUtil::norm2(cross((x2-x1),(x1-p1)))/
                              MetricUtil::norm2((x2-x1));
                if( dist <_threshold ) return true;
                else return false;*/
            } else {
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                if( dist <_threshold ) return true;
                // check if the point is inside the manifold box
                // project it onto the plane
                return isInsideOBB(p.p);
            }
        }

        /**
         * @brief fits a new manifold to the list of contact points
         * @param p
         */
        void fit(ContactPoint& p){
            using namespace boost::numeric;
            using namespace rw::math;
            // re-fit the bounding box
            ublas::matrix<double> covar( ublas::zero_matrix<double>(3, 3) );
            Vector3D<> c = p.p+_points[0].p+_points[1].p;
            covar(0,0) += _points[0].p(0)*_points[0].p(0) +
                          _points[1].p(0)*_points[1].p(0) +
                          p.p(0)+p.p(0);
            covar(1,1) += _points[0].p(1)*_points[0].p(1) +
                          _points[1].p(1)*_points[1].p(1) +
                          p.p(1)+p.p(1);
            covar(2,2) += _points[0].p(2)*_points[0].p(2) +
                          _points[1].p(2)*_points[1].p(2) +
                          p.p(2)+p.p(2);
            covar(0,1) += _points[0].p(0)*_points[0].p(1) +
                          _points[1].p(0)*_points[1].p(1) +
                          p.p(0)+p.p(1);
            covar(0,2) += _points[0].p(0)*_points[0].p(2) +
                          _points[1].p(0)*_points[1].p(2) +
                          p.p(0)+p.p(2);
            covar(1,2) += _points[0].p(1)*_points[0].p(2) +
                          _points[1].p(1)*_points[1].p(2) +
                          p.p(1)+p.p(2);

            for(int i=2; i<_nrOfContacts;i++){
                const Vector3D<> &point = _points[i].p;
                c += point;
                covar(0,0) += point(0)*point(0);
                covar(1,1) += point(1)*point(1);
                covar(2,2) += point(2)*point(2);
                covar(0,1) += point(0)*point(1);
                covar(0,2) += point(0)*point(2);
                covar(1,2) += point(1)*point(2);
            }
            const int n = _nrOfContacts+1;
            covar(0,0) = covar(0,0)-c(0)*c(0)/n;
            covar(1,1) = covar(1,1)-c(1)*c(1)/n;
            covar(2,2) = covar(2,2)-c(2)*c(2)/n;
            covar(0,1) = covar(0,1)-c(0)*c(1)/n;
            covar(0,2) = covar(0,2)-c(0)*c(2)/n;
            covar(1,2) = covar(1,2)-c(1)*c(2)/n;
            covar(1,0) = covar(0,1);
            covar(2,0) = covar(0,2);
            covar(2,1) = covar(1,2);

            typedef std::pair<ublas::matrix<double>,ublas::vector<double> > ResultType;
            //std::cout << "COVAR: " << covar << std::endl;
            ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

            // 4.1 create the rotationmatrix from the normalized eigenvectors
            // find max and the second maximal eigenvalue
            size_t maxEigIdx=2, midEigIdx=1, minEigIdx=0;
            double maxEigVal = res.second(maxEigIdx);
            double midEigVal = res.second(midEigIdx);
            double minEigVal = res.second(minEigIdx);
            if( maxEigVal < midEigVal ){
                std::swap(midEigVal,maxEigVal);
                std::swap(midEigIdx,maxEigIdx);
            }
            if( minEigVal>midEigVal ){
                std::swap(midEigVal,minEigVal);
                std::swap(midEigIdx,minEigIdx);
                if( midEigVal>maxEigVal ){
                    std::swap(midEigVal,maxEigVal);
                    std::swap(midEigIdx,maxEigIdx);
                }
            }
            // specify x and y axis, x will be the axis with largest spred
            Vector3D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx), res.first(2,maxEigIdx) );
            Vector3D<> midAxis( res.first(0,midEigIdx), res.first(1,midEigIdx), res.first(2,midEigIdx) );

            // make sure to turn the z-axis in the direction of the normal
            Vector3D<> crossAxis = cross(maxAxis,midAxis);
            if( dot(crossAxis,_points[_deepestIdx].n)<0 ){
                std::swap(maxAxis,midAxis);
                crossAxis = -crossAxis;
            }


            _normal = normalize(crossAxis);

            const Rotation3D<> rot(normalize(maxAxis),normalize(midAxis), _normal);
            const Rotation3D<> rotInv = inverse( rot );

            Vector3D<> max = rotInv * p.p;
            Vector3D<> min = max;
            for(int i = 0; i<_nrOfContacts; i++ ){
                const Vector3D<> prot = rotInv * _points[i].p;
                if( prot(0)>max(0) ) max(0) = prot(0);
                else if( prot(0)<min(0) ) min(0) = prot(0);
                if( prot(1)>max(1) ) max(1) = prot(1);
                else if( prot(1)<min(1) ) min(1) = prot(1);
                if( prot(2)>max(2) ) max(2) = prot(2);
                else if( prot(2)<min(2) ) min(2) = prot(2);
            }

            // compute halflength of box and its midpoint
            _t3d.P() = rot*( 0.5*(max+min));
            _t3d.R() = rot;
            _h = 0.5*(max-min);
            _normal = Vector3D<>(0,0,0);
            for(int i=0; i<_nrOfContacts;i++){
                _normal += _points[0].n;
            }
            _normal /= _nrOfContacts;

        }

        rw::math::Vector3D<> getNormal(){
            return _normal;
        }

        ContactPoint& getContact(int i){
            return _points[i];
        }

        rw::math::Transform3D<> getTransform(){ return _t3d;};

        rw::math::Vector3D<> getHalfLengths(){ return _h; };

    private:
        bool isInsideOBB(rw::math::Vector3D<>& p){

            rw::math::Vector3D<> pproj = inverse(_t3d) * p;
            if( fabs(pproj(0))<_h(0) && fabs(pproj(1))<_h(1) ){
                return true;
            }
            return false;
        }

        /**
         * The normal is an average of the normals of the contact normals until 3
         * or more contacts are available. Then the normal becomes the z axis in
         * the OBB that the contacts span
         */
        rw::math::Vector3D<> _normal; //
        rw::math::Transform3D<> _t3d; // transform of the obb
        rw::math::Vector3D<> _h; //halflengths off the obb
        // only used when 3 or more points are available
        ContactPoint _points[5];
        //Frame *_objA,*_objB;
        double _threshold,_sepThreshold;
        int _deepestIdx,_nrOfContacts;
    };

}
}

#endif /* OBRMANIFOLD_HPP_ */
