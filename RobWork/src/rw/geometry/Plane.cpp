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

#include "Plane.hpp"

#include "PlainTriMesh.hpp"


#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;

Q Plane::getParameters() const { 
    Q q(4);
    q(0) = _normal(0);
    q(1) = _normal(1);
    q(2) = _normal(2);
    q(3) = _d;
    return q;
}

TriMesh::Ptr Plane::createMesh(int resolution, double size) const  {
    size *= 0.5; //Scale s.t. the real size becomes size.

	// we find 4 points on the plane and create 2 triangles that represent the plane

    //Vector3D<> point = normalize(_normal) * _d/_normal.norm2();
	
    Vector3D<> otho;
    // find the orthogonal basis of the plane, eg xy-axis
    // use an axis to generate one orthogonal vector 
    if(fabs(angle(_normal, Vector3D<>::x())) > 0.001 && fabs(angle(_normal, Vector3D<>::x())) < Pi-0.001) {
        otho = cross(_normal, Vector3D<>::x());
    } else if( fabs(angle(_normal, Vector3D<>::y())) > 0.001 && fabs(angle(_normal, Vector3D<>::y())) < Pi-0.001) {
        otho = cross(_normal, Vector3D<>::y());
    } else if( fabs(angle(_normal, Vector3D<>::z())) > 0.001 && fabs(angle(_normal, Vector3D<>::z())) < Pi-0.001){
        otho = cross(_normal, Vector3D<>::z());
    }
    otho = normalize(otho);
    // and the final axis is then
    Vector3D<> ortho2 = normalize(cross(_normal, otho));


    Transform3D<> trans(-_normal*_d, Rotation3D<>(otho, ortho2, _normal));

    // now we generate the points in the two triangles
    rw::geometry::PlainTriMesh<>::Ptr mesh = ownedPtr( new rw::geometry::PlainTriMesh<>(2) );
    (*mesh)[0][0] = trans * (Vector3D<>::x()* size + Vector3D<>::y()* size);
    (*mesh)[0][1] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()* size);
    (*mesh)[0][2] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()*-size);

    (*mesh)[1][0] = trans * (Vector3D<>::x()*-size + Vector3D<>::y()*-size);
    (*mesh)[1][1] = trans * (Vector3D<>::x()* size + Vector3D<>::y()*-size);
    (*mesh)[1][2] = trans * (Vector3D<>::x()* size + Vector3D<>::y()* size);

    return mesh;

}

double Plane::refit( std::vector<rw::math::Vector3D<> >& data ){
	if( data.size()<3 )
		RW_THROW("Data size must be 3 or more!");

	if(data.size()==3){
		if( MetricUtil::dist2(data[1],data[0])<0.1 ||
				MetricUtil::dist2(data[2],data[1])<0.1 ||
				MetricUtil::dist2(data[0],data[2])<0.1){
			_normal = Vector3D<>(0,0,0);
			return 1000;
		}

		_normal = normalize( cross(data[1]-data[0],data[2]-data[0]) );
        if( _normal(2)<0 )
        	_normal = -_normal;
		const rw::math::Vector3D<>& p = data[0];
		_d = -_normal(0)*p(0) -_normal(1)*p(1) -_normal(2)*p(2);
	} else {

        using namespace boost::numeric;
        using namespace rw::math;

        Eigen::MatrixXd covar( Eigen::MatrixXd::Zero(3, 3) );
        Vector3D<> centroid(0,0,0);
        BOOST_FOREACH(Vector3D<> &v, data){
            centroid += v;
            for(size_t j=0;j<3;j++)
                for(size_t k=0;k<3;k++)
                    covar(j,k) += v(j)*v(k);
        }
        //std::cout << "COVAR: " << covar << std::endl;

        // 3. Compute Covariance matrix
        // 3.1 using the variables from 2.1 we create the covariance matrix
        for(size_t j=0;j<3;j++)
            for(size_t k=0;k<3;k++)
                covar(j,k) = covar(j,k)-centroid[j]*centroid[k]/data.size();
        Vector3D<> c = centroid/((double)data.size());
        // 4. get eigenvectors from the covariance matrix
        typedef std::pair<Eigen::MatrixXd, Eigen::VectorXd > ResultType;
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

        // compute the z axis as the cross product
        _normal = normalize( cross(maxAxis,midAxis) );
        // normal should allways point in the z direction
        if( _normal(2)<0 )
        	_normal = -_normal;
        _d = -_normal(0)*c(0) -_normal(1)*c(1) - _normal(2)*c(2);

	}

	// calculate the fit error as the squared mean over the distance of a point from the plane
	double sum = 0;
	BOOST_FOREACH(Vector3D<> &p, data){
		sum = Math::sqr( distance(p) );
	}
	sum /= data.size();


	return sum;
}



rw::math::Metric<Plane>::Ptr Plane::makeMetric(double angToDistWeight)
{
	return rw::common::ownedPtr(new PlaneMetric(angToDistWeight));
}

/*namespace {

	class PlaneMetric: Metric<Plane> {
	protected:
        double doDistance(const value_type& q) const = 0;

        double doDistance(const value_type& a, const value_type& b) const = 0;

        int doSize() const { return -1; }

	};
}*/



TriMesh::Ptr Plane::createMesh(int resolution) const {
	return createMesh(resolution, 100 /* Default plane size */);
}
