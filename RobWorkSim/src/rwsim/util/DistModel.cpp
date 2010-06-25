#include "DistModel.hpp"

#include <boost/foreach.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>

using namespace rw::math;
using namespace rwsim::util;

double DistModel::refit( std::vector<rw::math::Vector3D<> >& data ){
	if( data.size()==0 )
		RW_THROW("Data size must be 1 or more!");

	_invalid = false;

	using namespace boost::numeric;
	using namespace rw::math;

	Vector3D<> centroid(0,0,0);
	BOOST_FOREACH(Vector3D<> &v, data){
		centroid += v;
	}
	_center = centroid/data.size();

	double sum=0;
	BOOST_FOREACH(Vector3D<> &v, data){
		double d = MetricUtil::dist2(_center, v);
		sum += Math::sqr( 1/(d+1) );
	}

    return sum/data.size();
}
