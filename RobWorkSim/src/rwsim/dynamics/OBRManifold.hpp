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
#include <rw/math/Vector2D.hpp>

namespace rwsim {
namespace dynamics {

    /**
     * @brief Contact manifold based on Oriented Bounding Rectangle, so in 2D.
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
        bool addPoint(ContactPoint& p);

        ContactPoint& getDeepestPoint(){
            return _points[_deepestIdx];
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
        void fit(ContactPoint& p);

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
        rw::math::Vector2D<> _projPoints[6];
        bool _onBorderMap[4];
        //Frame *_objA,*_objB;
        double _threshold, _cosThreshold, _sepThreshold;
        int _deepestIdx,_nrOfContacts;
    };

}
}

#endif /* OBRMANIFOLD_HPP_ */
