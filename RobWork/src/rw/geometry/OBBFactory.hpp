/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_GEOMETRY_OBBFACTORY_HPP_
#define RW_GEOMETRY_OBBFACTORY_HPP_

#include <rw/geometry/Covariance.hpp>
#include <rw/math/EigenDecomposition.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/QHull3D.hpp>

#include "BV.hpp"
#include "OBB.hpp"

namespace rw {
namespace geometry {

    /**
     * @brief factory for computing tight fitting Oriented Bounding Boxes.
     */
    template<class T>
    class OBBFactory: public BVFactory<OBB<T> > {
    public:
        typedef enum{PCA, PCAHull, DITO14} FitMethod;

        OBBFactory(FitMethod method=DITO14):_method(method){}

        virtual ~OBBFactory(){};

        //! @brief create a BV
        virtual rw::geometry::OBB<T> makeBV(rw::geometry::TriMesh& mesh){
            switch(_method){
            case(DITO14): return makeDITO(mesh);
            case(PCA):return makePCA(mesh);
            case(PCAHull):return makePCAHull(mesh);
            default:
                RW_THROW("Unsupported fitting method!");
                break;
            }
            return makePCA(mesh);
        }

        rw::geometry::OBB<T> makeBV(rw::geometry::GeometryData& geom){
            rw::geometry::TriMesh* mesh = dynamic_cast<rw::geometry::TriMesh*>(&geom);
            if( mesh!=NULL ){
                return makeBV(*mesh);
            }
            // TODO: this might be very inefficient
            rw::common::Ptr<rw::geometry::TriMesh> geommesh = geom.getTriMesh(false);
            return makeBV(*geommesh);
        }

        rw::geometry::OBB<T> makeBV(rw::geometry::Primitive& geom){
            // TODO: this might be very inefficient
            rw::common::Ptr<rw::geometry::TriMesh> geommesh = geom.getTriMesh(false);
            return makeBV(*geommesh);
        }

        rw::geometry::OBB<T> makeDITO(rw::geometry::TriMesh& mesh);

        /**
         * @brief computes covariance over vertices in mesh and calculates the
         * eigen vectors of the covariance and use this as axes in the bounding box
         * @param mesh
         * @return a tight fitting bounding box
         */
        rw::geometry::OBB<T> makePCA(rw::geometry::TriMesh& mesh);

        /**
         * @brief computes covariance over the vertices of the convex hull
         * of the  mesh and calculates the
         * eigen vectors of the covariance and use this as axes in the bounding box
         * @param mesh
         * @return a tight fitting bounding box
         */
        rw::geometry::OBB<T> makePCAHull(rw::geometry::TriMesh& mesh);

    private:
        FitMethod _method;
    };

    template<class T>
    rw::geometry::OBB<T> OBBFactory<T>::makeDITO(rw::geometry::TriMesh& mesh){
        return makePCAHull(mesh);
    }

    template<class T>
    rw::geometry::OBB<T> OBBFactory<T>::makePCA(rw::geometry::TriMesh& mesh){
        //return rw::geometry::OBB<T>::buildTightOBB(mesh);

        using namespace rw::math;
        //std::cout << "\nMesh size: " << mesh.size() << "\n";
        Covariance<> covar;
        TriMesh::VerticeIterator iter(mesh);
        covar.doInitialize<TriMesh::VerticeIterator,3>(iter,iter);
        EigenDecomposition<> eigend = covar.eigenDecompose();
        // the eigendecomposition has the eigen vectors and value.
        // we want the x-axis of the OBB to be aligned with the largest eigen vector.
        eigend.sort();
        Vector3D<> axisX( eigend.getEigenVector(2) );
        Vector3D<> axisY( eigend.getEigenVector(1) );
        Vector3D<> axisZ = cross(axisX,axisY);
        // so now we can form the basis of the rotation matrix of the OBB
        Rotation3D<> rot(normalize(axisX),normalize(axisY),normalize(axisZ));
        Rotation3D<> rotInv = inverse( rot );
        // last we need to find the maximum and minimum points in the mesh to determine
        // the bounds (halflengts) of the OBB
        Triangle<> t = mesh.getTriangle(0);
        Vector3D<> p = rotInv * cast<T>(t[0]);
        Vector3D<> max=p, min=p;
        Triangle<> tri;
        for(size_t i=0;i<mesh.getSize();i++){
            mesh.getTriangle(i, tri);
            for(int pidx=0;pidx<3; pidx++){
                Vector3D<> p = rotInv * cast<T>(tri[pidx]);
                for(int j=0; j<3; j++){
                    if( p(j)>max(j) ) max(j) = p(j);
                    else if( p(j)<min(j) ) min(j) = p(j);
                }
            }
        }
        Vector3D<> midPoint = rot*( 0.5*(max+min));
        Vector3D<> halfLength = 0.5*(max-min);
        Transform3D<> trans(midPoint,rot);
        return OBB<T>(trans, halfLength);
    }

    template<class T>
    rw::geometry::OBB<T> OBBFactory<T>::makePCAHull(rw::geometry::TriMesh& meshInput){
        using namespace rw::math;
        
        QHull3D hullgen;

        IndexedTriMeshN0D::Ptr mesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0D>(meshInput);
        TriMesh::Ptr pmesh;
        if(mesh->size()<5){
            pmesh = mesh;
        } else {
            // build the convex hull
            hullgen.rebuild(mesh->getVertices() );
            pmesh = hullgen.toTriMesh();
        }

        //std::cout << "\nMesh size: " << mesh.size() << "\n";
        Eigen::MatrixXd covarM = Eigen::MatrixXd(3,3);
        Vector3D<> centroidSum(0,0,0);
        double aSum= 0;
        for(size_t i=0;i<pmesh->size();i++){
            Triangle<> tri = pmesh->getTriangle(i);
            const Vector3D<>& p = tri[0];
            const Vector3D<>& q = tri[1];
            const Vector3D<>& r = tri[2];
            double a = tri.calcArea();
            aSum += a;

            Vector3D<> c = (p+q+r)*1.0/3.0;
            centroidSum += c*a;

            double ascale = (a/12.0);
            covarM(0,0) += ascale*(9*c[0]*c[0] + p[0]*p[0] + q[0]*q[0] + r[0]*r[0] );
            covarM(1,1) += ascale*(9*c[1]*c[1] + p[1]*p[1] + q[1]*q[1] + r[1]*r[1] );
            covarM(2,2) += ascale*(9*c[2]*c[2] + p[2]*p[2] + q[2]*q[2] + r[2]*r[2] );
            covarM(0,1) += ascale*(9*c[0]*c[1] + p[0]*p[1] + q[0]*q[1] + r[0]*r[1] );
            covarM(0,2) += ascale*(9*c[0]*c[2] + p[0]*p[2] + q[0]*q[2] + r[0]*r[2] );
            covarM(1,2) += ascale*(9*c[1]*c[2] + p[1]*p[2] + q[1]*q[2] + r[1]*r[2] );
        }
        double ah_inv = (1.0/aSum);
        centroidSum = centroidSum * ah_inv;

        covarM(0,0) = ah_inv * covarM(0,0) - centroidSum[0]*centroidSum[0];
        covarM(1,1) = ah_inv * covarM(1,1) - centroidSum[1]*centroidSum[1];
        covarM(2,2) = ah_inv * covarM(2,2) - centroidSum[2]*centroidSum[2];
        covarM(0,1) = ah_inv * covarM(0,1) - centroidSum[0]*centroidSum[1];
        covarM(0,2) = ah_inv * covarM(0,2) - centroidSum[0]*centroidSum[2];
        covarM(1,2) = ah_inv * covarM(1,2) - centroidSum[1]*centroidSum[2];

        covarM(1,0) = covarM(0,1);
        covarM(2,0) = covarM(0,2);
        covarM(2,1) = covarM(1,2);

        Covariance<> covar(covarM);
        EigenDecomposition<> eigend = covar.eigenDecompose();
        // the eigendecomposition has the eigen vectors and value.
        // we want the x-axis of the OBB to be aligned with the largest eigen vector.
        eigend.sort();
        Vector3D<> axisX( eigend.getEigenVector(2) );
        Vector3D<> axisY( eigend.getEigenVector(1) );
        Vector3D<> axisZ = cross(axisX,axisY);
        // so now we can form the basis of the rotation matrix of the OBB
        Rotation3D<> rot(normalize(axisX),normalize(axisY),normalize(axisZ));
        Rotation3D<> rotInv = inverse( rot );
        //std::cout << rot << std::endl;

        // last we need to find the maximum and minimum points in the mesh to determine
        // the bounds (halflengts) of the OBB
        Triangle<> t = pmesh->getTriangle(0);
        Vector3D<> p = rotInv * cast<T>(t[0]);
        Vector3D<> max=p, min=p;
        for(size_t i=0;i<pmesh->getSize();i++){
            Triangle<> tri = pmesh->getTriangle(i);
            for(int pidx=0;pidx<3; pidx++){
                Vector3D<> p = rotInv * cast<T>(tri[pidx]);
                for(int j=0; j<3; j++){
                    if( p(j)>max(j) ) max(j) = p(j);
                    else if( p(j)<min(j) ) min(j) = p(j);
                }
            }
        }
        Vector3D<> midPoint = rot*( 0.5*(max+min));
        Vector3D<> halfLength = 0.5*(max-min);
        Transform3D<> trans(midPoint,rot);
        return OBB<T>(trans, halfLength);
    }


}
}

#endif
