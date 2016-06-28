#ifndef RW_GEOMETRY_BSPHERE_HPP_
#define RW_GEOMETRY_BSPHERE_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Constants.hpp>

#include "BV.hpp"

namespace rw {
namespace geometry {
	class TriMesh;

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
        BSphere(T radius=1.0):
            _p3d( rw::math::Vector3D<T>(0,0,0) ),
            _radius(radius)
        {}

        BSphere(const rw::math::Vector3D<T>& pos, T radius=1.0):
            _p3d( rw::math::Vector3D<T>(0,0,0) ),
            _radius(radius)
        {}

        inline const rw::math::Vector3D<T>& getPosition() const {
            return _p3d;
        }

        inline void setPosition(const rw::math::Vector3D<T>& p3d) { _p3d = p3d; }

        inline const T& getRadius() const {
            return _radius;
        }

        inline const T getRadiusSqr() const {
            return _radius*_radius;
        }

        inline T calcArea() const {
            return (T)(4.0*rw::math::Pi*_radius*_radius);
        }

        inline T calcVolume() const {
            return (T)(4.0/3.0*rw::math::Pi*_radius*_radius*_radius);
        }

        /**
         * @brief Ouputs BSphere to stream
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

}

//! define traits of the OBB
template<typename T> struct Traits<geometry::BSphere<T> >{ typedef T value_type; };

}
#endif /*OBB_HPP_*/
