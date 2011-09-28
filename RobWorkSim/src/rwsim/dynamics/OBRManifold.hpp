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
         * @param thres [in] the angle threshold in radians. Threshold of angle between
         * contact normals
         * @param sepThres [in] the max seperating distance in meter between contacting points
         */
        OBRManifold(double thres = 0.03, double sepThres = 0.01):
            _threshold(thres),
            _cosThreshold( fabs( std::cos(thres) ) ),
            _sepThreshold(sepThres),
            _deepestIdx(0),
            _nrOfContacts(0)
        {};

        virtual ~OBRManifold(){};



        /**
         * @brief adds and updates the manifold with a new point
         * @param p
         */
        bool addPoint(ContactPoint& p){
            using namespace rw::math;
            //if( !inManifold(p) )
            //    RW_THROW("Not in manifold");

            // handle if we have less than 5 points
            if(_nrOfContacts==5){
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
                _points[minPenIdx] = p;
                if( _points[_deepestIdx].penetration<p.penetration )
                    _deepestIdx = minPenIdx;
                return true;
            } else if(_nrOfContacts>1) {
                // only add a point if its on the contact manifold plane
                //double dist = dot(p.p,_normal)+dot(_normal, _points[0].p);
                double dist = fabs( dot(p.n,_normal) );
                if(acos(dist)>_threshold)
                    return false;
                // now also test if the point is close enough to the contacting plane
                //double d = -dot(_normal, _points[0].p);
                //dist = fabs( dot(_normal,p.p)+d );
                //if(dist>_sepThreshold)
                //    return false;

                // with at least 3 points we can create a manifold plane
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
                    std::cout << acos(dist) << ">" << _threshold << std::endl;
                    return false;
                }

                _normal = (p.n + _points[0].n)/2;
                _points[_nrOfContacts] = p;
                _t3d.P() = (p.p + _points[0].n)/2;
                if( _points[_deepestIdx].penetration<p.penetration )
                    _deepestIdx = _nrOfContacts;
                _nrOfContacts++;
            } else {
                _normal = p.n;
                _points[0] = p;
                _t3d.P() = p.p;
                _deepestIdx = 0;
                _nrOfContacts++;
            }
            return true;
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
                // check normal


                // check distance to the deepest point
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                //std::cout << "1: Dist too point: " << dist << std::endl;
                if( dist <_sepThreshold ) return true;
                else return false;
            } else if( _nrOfContacts == 2 ){
                // check distance to the line
                const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
                //std::cout << "1: Dist too point: " << dist << std::endl;
                if( dist <_sepThreshold ) return true;
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
                if( dist <_sepThreshold ) return true;
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
            int x_max = _nrOfContacts, x_min = _nrOfContacts,
                    y_max = _nrOfContacts, y_min = _nrOfContacts,
                    z_max = _nrOfContacts, z_min = _nrOfContacts;
            for(int i = 0; i<_nrOfContacts; i++ ){
                const Vector3D<> prot = rotInv * _points[i].p;
                if( prot(0)>max(0) ) {max(0) = prot(0); x_max = i;}
                else if( prot(0)<min(0) ) {min(0) = prot(0); x_min = i;}
                if( prot(1)>max(1) ) {max(1) = prot(1); y_max = i;}
                else if( prot(1)<min(1) ) {min(1) = prot(1); y_min=i;}
                if( prot(2)>max(2) ) {max(2) = prot(2); z_max = i;}
                else if( prot(2)<min(2) ) {min(2) = prot(2); z_min = i;}
            }
            for(int i=0;i<6;i++) _onBorderMap[i] = false;
            _onBorderMap[x_min] = true;
            _onBorderMap[x_max] = true;
            _onBorderMap[y_min] = true;
            _onBorderMap[y_max] = true;
            //_onBorderMap[z_min] = true;
            //_onBorderMap[z_max] = true;

            // compute halflength of box and its midpoint
            _t3d.P() = rot*( 0.5*(max+min));
            _t3d.R() = rot;
            _h = 0.5*(max-min);
            //_normal = p.n;
            //for(int i=0; i<_nrOfContacts;i++){
            //    _normal += _points[0].n;
            //}
            //_normal /= _nrOfContacts+1;

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
        void updateDeepestPoint(){

        }

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
        //rw::math::Vector3D<> _manifoldNormal; // the normal to the manifold
        rw::math::Transform3D<> _t3d; // transform of the obb
        rw::math::Vector3D<> _h; //halflengths off the obb
        // only used when 3 or more points are available
        ContactPoint _points[5];
        bool _onBorderMap[6];
        //Frame *_objA,*_objB;
        double _threshold,_sepThreshold, _cosThreshold;
        int _deepestIdx,_nrOfContacts;
    };

}
}

#endif /* OBRMANIFOLD_HPP_ */
