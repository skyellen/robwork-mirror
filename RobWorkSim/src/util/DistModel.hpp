/*
 * PlaneModel.hpp
 *
 *  Created on: Dec 17, 2008
 *      Author: jimali
 */

#ifndef DISTMODEL_HPP_
#define DISTMODEL_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

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

#endif /* DistModel_HPP_ */
