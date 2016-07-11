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

#include "PlaneModel.hpp"

#include <boost/foreach.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/common/macros.hpp>

using namespace rw::math;
using namespace rwsim::util;

double PlaneModel::refit( std::vector<rw::math::Vector3D<> >& data ){
	if( data.size()<3 )
		RW_THROW("Data size must be 3 or more!");

	if(data.size()==3){
		if( MetricUtil::dist2(data[1],data[0])<0.1 ||
				MetricUtil::dist2(data[2],data[1])<0.1 ||
				MetricUtil::dist2(data[0],data[2])<0.1){
			_n = Vector3D<>(0,0,0);
			return 1000;
		}

		_n = normalize( cross(data[1]-data[0],data[2]-data[0]) );
        if( _n(2)<0 )
        	_n = -_n;
		const rw::math::Vector3D<>& p = data[0];
		_d = -_n(0)*p(0) -_n(1)*p(1) -_n(2)*p(2);
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
        typedef std::pair<Eigen::MatrixXd,Eigen::VectorXd> ResultType;
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
        _n = normalize( cross(maxAxis,midAxis) );
        // normal should allways point in the z direction
        if( _n(2)<0 )
        	_n = -_n;
        _d = -_n(0)*c(0) -_n(1)*c(1) - _n(2)*c(2);

	}

	// calculate the fit error as the squared mean over the distance of a point from the plane
	double sum = 0;
	BOOST_FOREACH(Vector3D<> &p, data){
		const double fitE = fitError(p);
		sum = fitE*fitE;
	}
	sum /= data.size();


	return sum;
}
