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
#include <boost/concept_check.hpp>

#include "rw/math/Rotation2D.hpp"

#include "BeamGeometry.hpp"
#include "BeamObstaclePlane.hpp"

#include "ModRusselBeamBase.hpp"

namespace rwlibs {
namespace softbody {

class ModRusselBeam : public ModRusselBeamBase
{
    public:
	
	ModRusselBeam(
        boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
        boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
        int M
	);
	
	~ModRusselBeam() {};
	
	
    public:
	
	void objective(const boost::numeric::ublas::vector<double>& x,
		       double& f,
		       boost::numeric::ublas::vector<double>& df,
		       boost::numeric::ublas::matrix<double>& ddf); 
		       
	void equalityConstraints(const boost::numeric::ublas::vector<double>& x,
	size_t idx,
	double& g,
	boost::numeric::ublas::vector<double>& dg,
	boost::numeric::ublas::matrix<double>& ddg); 
	
// 	void setInEqualityIntegralConstraint(
// 	const boost::numeric::ublas::vector< double >& x,
// 					   size_t idx, 
// 					   boost::numeric::ublas::vector< double >& h, 
// 					   boost::numeric::ublas::matrix< double >& dh,
// 					   boost::numeric::ublas::matrix< double >& ddh
// 					   );
    
    void setInEqualityIntegralConstraintPoint(
                        const boost::numeric::ublas::vector< double >& x,
                       size_t idx, 
                       boost::numeric::ublas::vector< double >& h, 
                       boost::numeric::ublas::matrix< double >& dh,
                       boost::numeric::ublas::matrix< double >& ddh,
                       int pIdx,
                       int hBase
                       );
    
    void setHingeConstraintPointY(
                        const boost::numeric::ublas::vector< double >& x,
                       size_t idx, 
                       boost::numeric::ublas::vector< double >& h, 
                       boost::numeric::ublas::matrix< double >& dh,
                       boost::numeric::ublas::matrix< double >& ddh,
                       int pIdx,
                       int hBase
                       );
					   
	void setInEqualityNoUpwardsEtaConstraint(
	const boost::numeric::ublas::vector< double >& x,
						size_t idx, 
						boost::numeric::ublas::vector< double >& h, 
						boost::numeric::ublas::matrix< double >& dh,
						boost::numeric::ublas::matrix< double >& ddh,
                        int hBase
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
    
    double f_elastic(const boost::numeric::ublas::vector< double >& x);

    void df(boost::numeric::ublas::vector<double> &res, const boost::numeric::ublas::vector<double>& x);

    void ddf_banded2(boost::numeric::ublas::matrix<double> &res, const boost::numeric::ublas::vector<double>& x);  
									   
	void solve(boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector<double> &U, boost::numeric::ublas::vector<double> &V);


	void integrateAngleU(boost::numeric::ublas::vector<double> &U, const boost::numeric::ublas::vector<double> &avec);
    void integrateAngleV(boost::numeric::ublas::vector<double> &V, const boost::numeric::ublas::vector<double> &avec);

    
    rw::math::Transform3D<> get_planeTbeam(void) const;
    double get_yTCP(void) const;
    double get_thetaTCP(void) const;
    double get_uxTCPy(void) const;
    double get_uyTCPy(void) const;
    
    void setUseNoUpwardConstraint(bool val);
    void setUseHingeConstraint(bool val);
    
    void setAccuracy(double acc);
    void setMuStart(double muStart);
    void setMuDecrementFactor(double decFactor);
    
    void set_nIntegralConstraints(int nIntegralConstraints);
    int get_nIntegralConstraints(void) const;
	
	friend std::ostream& operator<<(std::ostream& out, const ModRusselBeam& obj) {
	    std::stringstream str;
        
        const rw::math::Transform3D<> planeTbeam = obj.get_planeTbeam();
            
        double yTCP = obj._obstaclePtr->get_yTCP(planeTbeam);
        double thetaTCP = obj._obstaclePtr->get_thetaTCP(planeTbeam);
        double g1 = obj._geomPtr->g1();
        double g2 = obj._geomPtr->g2();
        const double uxTCPy =  obj.get_uxTCPy();
        const double uyTCPy = obj.get_uyTCPy();
	        
	    str << "ModRusselBeam {M:" << obj.getM() << ", g1: " << g1 << ", g2:" << g2 << ", uxTCPy: " << uxTCPy << ", uyTCPy: " << uyTCPy << ", yTCP: " << yTCP << ", thetaTCP: " << thetaTCP << ", accuracy: " << obj._accuracy << ", useNoUpwardConstraint: " << obj._useNoUpwardConstraint << "}";
	    
	    return out << str.str();
	};
	
    private:
	boost::shared_ptr< BeamGeometry > _geomPtr; 
	boost::shared_ptr< BeamObstaclePlane > _obstaclePtr; 
	int _M;
	double _accuracy;
	boost::numeric::ublas::vector<double> 	_a;
	boost::numeric::ublas::vector<double> 	_da; 
	
	bool _useNoUpwardConstraint;	
    
    int _nIntegralConstraints;
    
    bool _useHingeConstraint;
    std::vector<int> _integralConstraintIdxList;
    
    double _muStart;
    double _muDec;
};
}}

#endif // MODRUSSELBEAM_HPP
