/*
 * PlaneModel.hpp
 *
 *  Created on: Dec 17, 2008
 *      Author: jimali
 */

#ifndef PLANEMODEL_HPP_
#define PLANEMODEL_HPP_

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Math.hpp>

#include <vector>
#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

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

	// aux variables
	double _nNorm2;
};

#endif /* PLANEMODEL_HPP_ */
