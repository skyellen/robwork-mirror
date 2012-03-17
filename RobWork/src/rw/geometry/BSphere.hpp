#ifndef RW_GEOMETRY_BSPHERE_HPP_
#define RW_GEOMETRY_BSPHERE_HPP_

#include <vector>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/PlainTriMesh.hpp>

//#include <rw/geometry/GiftWrapHull3D.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include "BV.hpp"

namespace rw {
namespace geometry {


    /**
     * @brief class representing an Oriented Bounding Box (OBB)
     */

    template <class T=double>
    class BSphere: public BV<BSphere<T> > {
    public:
        typedef T value_type;

        /**
         * @brief constructor
         */
        BSphere(double radius=1.0):
            _p3d( rw::math::Vector3D<T>(0,0,0) ),
            _radius(radius)
        {}

        BSphere(const rw::math::Vector3D<T>& pos,double radius=1.0):
            _p3d( rw::math::Vector3D<T>(0,0,0) ),
            _radius(radius)
        {}

        inline const rw::math::Vector3D<T>& getPosition() const {
            return _p3d;
        }

        inline void setPosition(rw::math::Vector3D<T> p3d) { _p3d = p3d; }

        inline const T& getRadius() const {
            return _radius;
        }

        inline const T getRadiusSqr() const {
            return _radius*_radius;
        }

        inline T calcArea() const {
            return 4.0*rw::math::Pi*_radius*_radius;
        }

        inline T calcVolumne() const {
            return 4.0/3.0*rw::math::Pi*_radius*_radius*_radius;
        }

        /**
         * @brief Ouputs OBB to stream
         * @param os [in/out] stream to use
         * @param obb [in] oriented bounding box
         * @return the resulting stream
         */
        friend std::ostream& operator<<(std::ostream& os, const BSphere<T>& sphere){
            return os <<" BSphere{" << sphere._p3d << "," << sphere._radius << "}";
        }

        /**
         * @brief fit a sphere in $O(n)$ to a triangle mesh using Principal Component Analysis (PCA)
         * where the eigen values of the vertices are used to compute the center of the sphere
         * using the vector with the maximum spread (largest eigenvalue).
         * @param tris [in] input mesh
         * @return
         */
        static BSphere<T> fitEigen(const rw::geometry::TriMesh& tris);

        //static BSphere<T> fitRitter(const rw::geometry::TriMesh& tris);
    private:
        rw::math::Vector3D<T> _p3d;
        T _radius;

    };

    template <class T>
    BSphere<T> BSphere<T>::fitEigen(const rw::geometry::TriMesh& tris){
        using namespace boost::numeric;
        using namespace rw::math;
        using namespace rw::geometry;
        // 1. Compute convex hull
        //std::cout << "build hull from: " << tris.size() <<  std::endl;
        const int nrOfTris = tris.getSize();

        // 2. Compute centroid for convex hull
        // 2.1 centroid is computed using the triangles of the convex hull
        //ublas::bounded_matrix<T,3,3> covar;
        ublas::matrix<T> covar( ublas::zero_matrix<T>(3, 3) );
        Vector3D<T> centroid(0,0,0);

        // we only use triangle centers the vertices directly
        for(int i = 0; i<nrOfTris; i++ ){
            //std::cout << "i" << i << std::endl;
            // calc triangle centroid
            rw::geometry::Triangle<> t = tris.getTriangle(i);
            centroid += cast<T>( t[0]+t[1]+t[2] );
            for(size_t j=0;j<3;j++)
                for(size_t k=0;k<3;k++)
                    covar(j,k) += t[0](j)*t[0](k) +
                    t[1](j)*t[1](k) +
                    t[2](j)*t[2](k);
        }

        int n = nrOfTris*3;

        // 3. Compute Covariance matrix
        // 3.1 using the variables from 2.1 we create the covariance matrix
        for(size_t j=0;j<3;j++)
            for(size_t k=0;k<3;k++)
                covar(j,k) = covar(j,k)-centroid[j]*centroid[k]/n;

        // 4. get eigenvectors from the covariance matrix
        typedef std::pair<ublas::matrix<T>,ublas::vector<T> > ResultType;
        //std::cout << "COVAR: " << covar << std::endl;
        ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

        // 4.1 create the rotationmatrix from the normalized eigenvectors
        // find max and the second maximal eigenvalue
        size_t maxEigIdx=2, midEigIdx=1, minEigIdx=0;
        T maxEigVal = res.second(maxEigIdx);
        T midEigVal = res.second(midEigIdx);
        T minEigVal = res.second(minEigIdx);
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
        Vector3D<T> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx), res.first(2,maxEigIdx) );
        Vector3D<T> midAxis( res.first(0,midEigIdx), res.first(1,midEigIdx), res.first(2,midEigIdx) );
        // compute the z axis as the cross product
        Vector3D<T> crossAxis = cross(maxAxis,midAxis);

        // generate the rotation matrix
        Rotation3D<T> rot(normalize(maxAxis),normalize(midAxis),normalize(crossAxis));
        // 5. find extreme vertices relative to eigenvectors
        // we use the generated rotation matrix to rotate each point and then save
        // max and min values of each axis
        Rotation3D<T> rotInv = inverse( rot );

        Triangle<> t = tris.getTriangle(0);
        Vector3D<T> p = rotInv * cast<T>(t[0]);
        Vector3D<T> max=p, min=p;
        for(int i = 0; i<nrOfTris; i++ ){
            Triangle<> tri = tris.getTriangle(i);
            for(int pidx=0;pidx<3; pidx++){
                Vector3D<T> p = rotInv * cast<T>(tri[pidx]);
                for(int j=0; j<3; j++){
                    if( p(j)>max(j) ) max(j) = p(j);
                    else if( p(j)<min(j) ) min(j) = p(j);
                }
            }
        }

        // 6. use them to generate OBB
        //std::cout << "Max-Min: " << (max-min) << std::endl;

        // compute halflength of box and its midpoint
        Vector3D<T> midPoint = rot*( 0.5*(max+min));
        Vector3D<T> halfLength = 0.5*(max-min);
        //std::cout << "halflength: " << halfLength << std::endl;
        //std::cout << "midpoint: " << midPoint << std::endl;
        //Transform3D<T> trans(midPoint,rot);
        //std::cout << "Trans mid: " << trans.P() << std::endl;
        return BSphere<T>(midPoint, halfLength.norm2() );
    };

}

//! define traits of the OBB
template<typename T> struct Traits<geometry::BSphere<T> >{ typedef T value_type; };

}
#endif /*OBB_HPP_*/
