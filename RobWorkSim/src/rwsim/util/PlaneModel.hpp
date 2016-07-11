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

#ifndef RWSIM_UTIL_PLANEMODEL_HPP_
#define RWSIM_UTIL_PLANEMODEL_HPP_

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

namespace rwsim {
namespace util {


	class PlaneModel {
	public:
		PlaneModel( ):_d(0),_n(0,0,1){

		}

		PlaneModel( std::vector<rw::math::Vector3D<> >& data ){
			refit(data);
		}

		double refit( std::vector<rw::math::Vector3D<> >& data );

		bool invalid(){ return _n.norm2()<0.9; }; // the normal must be unt length

		/**
		 * @brief the
		 */
		double fitError( rw::math::Vector3D<>& p ){
			return fabs( _n(0)*p(0)+_n(1)*p(1)+_n(2)*p(2) + _d );
		}

		void print(){
			std::cout << " " << _d << "  " << _n << " " << std::endl;
		}

		const rw::math::Vector3D<>& getNormal()const { return _n; };
		const double& getD()const { return _d; };

		bool same( PlaneModel& model, double thres){
			if( (rw::math::MetricUtil::dist2(_n, model.getNormal()) > thres))
				return false;

			if( fabs(_d-model.getD())> 0.1 )
				return false;

			return true;
		}

		static int getMinReqData(){ return 3;};

	private:
		double _d;
		rw::math::Vector3D<> _n; // unit vector of the plane
	};
}
}
#endif /* PLANEMODEL_HPP_ */
