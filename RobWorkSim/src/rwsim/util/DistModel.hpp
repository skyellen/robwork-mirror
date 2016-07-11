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

#ifndef RWSIM_UTIL_DISTMODEL_HPP_
#define RWSIM_UTIL_DISTMODEL_HPP_

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

namespace rwsim {
namespace util {
	//! @addtogroup rwsim_util
	//! @{
	class DistModel {
	public:
		DistModel( ):_center(0,0,0),_invalid(true){

		}

		DistModel( std::vector<rw::math::Vector3D<> >& data ){
			refit(data);
		}

		double refit( std::vector<rw::math::Vector3D<> >& data );

		bool invalid(){ return _invalid; };

		/**
		 * @brief the
		 */
		double fitError( rw::math::Vector3D<>& p ){
			return rw::math::MetricUtil::dist2(_center,p);
		}

		void print(){
			std::cout << " " << _center << " " << std::endl;
		}

		rw::math::Vector3D<>& getCenter(){ return _center; };

		bool same( DistModel& model, double thres){
			if( (rw::math::MetricUtil::dist2(_center, model.getCenter()) > thres*1.5))
				return false;

			return true;
		}

		static int getMinReqData(){ return 1; };

	private:

		rw::math::Vector3D<> _center; // unit vector of the plane
		bool _invalid;
	};
}
}
#endif /* DistModel_HPP_ */
