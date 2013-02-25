/*
    Copyright [yyyy] [name of copyright owner]

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

*/

#ifndef MODRUSSELBEAM_HPP
#define MODRUSSELBEAM_HPP

#include <cstddef>

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "rw/math/Rotation2D.hpp"

// using namespace boost::numeric::ublas;
#include "BeamGeometry.hpp"

namespace rwlibs {
namespace softbody {

class ModRusselBeam
{
    public:
	
	ModRusselBeam(
	boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
	int M,
		      double yTCP,
		      double thetaTCP,
		      double accuracy,
		      bool useNoUpwardConstraint
	);
	
	~ModRusselBeam() {};
	
	// 	rw::trajectory::Vector3DPath solve(rw::trajectory::Vector3DTrajectory::Ptr trajectory, int cnt, double minBendSeperation);
	
	
    public:
	// 	rw::trajectory::Vector3DTrajectory::Ptr _trajectory;
	// 	double _mbs; //Minimum Bend Seperation
	// 	rw::math::Vector3D<> r(double s);
	
	
	void objective(const boost::numeric::ublas::vector<double>& x,
		       double& f,
		       boost::numeric::ublas::vector<double>& df,
		       boost::numeric::ublas::matrix<double>& ddf); 
		       
	void equalityConstraints(const boost::numeric::ublas::vector<double>& x,
	size_t idx,
	double& g,
	boost::numeric::ublas::vector<double>& dg,
	boost::numeric::ublas::matrix<double>& ddg); 
	
	void setInEqualityVIntegralConstraint(
	const boost::numeric::ublas::vector< double >& x,
					   size_t idx, 
					   boost::numeric::ublas::vector< double >& h, 
					   boost::numeric::ublas::matrix< double >& dh,
					   boost::numeric::ublas::matrix< double >& ddh
					   );
					   
	void setInEqualityNoUpwardsEtaConstraint(
	const boost::numeric::ublas::vector< double >& x,
						size_t idx, 
						boost::numeric::ublas::vector< double >& h, 
						boost::numeric::ublas::matrix< double >& dh,
						boost::numeric::ublas::matrix< double >& ddh
						);					   

	void inEqualityConstraints(const boost::numeric::ublas::vector<double>& x,
	size_t idx,
	boost::numeric::ublas::vector<double>& h,
	boost::numeric::ublas::matrix<double>& dh,
	boost::numeric::ublas::matrix<double>& ddh);

	int getN(void) const;
	
	int getM(void) const;
	
	double get_h(void) const;
	
	double f(const boost::numeric::ublas::vector< double >& x);

	boost::numeric::ublas::vector<double> df(const boost::numeric::ublas::vector<double>& x);

	boost::numeric::ublas::matrix<double> ddf(const boost::numeric::ublas::vector<double>& x); 	
	
	double diff_i(const boost::numeric::ublas::vector< double >& x, const int i);
	
	boost::numeric::ublas::matrix<double> ddf_banded(const boost::numeric::ublas::vector<double>& x); 	
	boost::numeric::ublas::matrix<double> ddf_banded2(const boost::numeric::ublas::vector<double>& x); 	
									   
	void solve(boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector<double> &U, boost::numeric::ublas::vector<double> &V);
	
	boost::numeric::ublas::vector<double> integrateAngleU(const boost::numeric::ublas::vector<double> a)  ;
	boost::numeric::ublas::vector<double> integrateAngleV(const boost::numeric::ublas::vector<double> a)  ;
	
	double get_uxTCPy() { return _uxTCPy; };
	double get_uyTCPy() { return _uyTCPy; };
	
	const rw::math::Rotation2D<double> & getRot() { return _rot; };
	
	friend std::ostream& operator<<(std::ostream& out, const ModRusselBeam& obj) {
	    std::stringstream str;
	        
	    str << "ModRusselBeam {M:" << obj.getM() << ", yTCP: " << obj._yTCP << ", thetaTCP: " << obj._thetaTCP << ", accuracy: " << obj._accuracy << ", useNoUpwardConstraint: " << obj._useNoUpwardConstraint << "}";
	    
	    return out << str.str();
	};
	
    private:
	boost::shared_ptr< BeamGeometry > _geomPtr; // hvornår bliver denne ref. fucket up?
	int _M;
	double _yTCP;
	double _thetaTCP;
	double _accuracy;
	boost::numeric::ublas::vector<double> 	_a;
	boost::numeric::ublas::vector<double> 	_da; 
	
	rw::math::Rotation2D<double> 	_rot;
	double _uxTCPy;
	double _uyTCPy;
	
	double _G;
	double 	_g1;
	double _g2;
	
	bool _useNoUpwardConstraint;
	
};
}}

#endif // MODRUSSELBEAM_HPP
