/*
Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,

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

#ifndef RWLIBS_SOFTBODY_MODRUSSELBEAM_HPP
#define RWLIBS_SOFTBODY_MODRUSSELBEAM_HPP

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
/** @addtogroup softbody */
/*@{*/
/**
 * @brief Implementation of the Modified Russel beam problem (Deprecated!)
 *
 * @deprecated
 **/
class ModRusselBeam : public ModRusselBeamBase {
public:

    ModRusselBeam (
        boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
        boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
        int M
    );

    ~ModRusselBeam() {};


public:

    void objective ( const boost::numeric::ublas::vector<double>& x,
                     double& f,
                     boost::numeric::ublas::vector<double>& df,
                     boost::numeric::ublas::matrix<double>& ddf );

    void equalityConstraints ( const boost::numeric::ublas::vector<double>& x,
                               size_t idx,
                               double& g,
                               boost::numeric::ublas::vector<double>& dg,
                               boost::numeric::ublas::matrix<double>& ddg );

// 	void setInEqualityIntegralConstraint(
// 	const boost::numeric::ublas::vector< double >& x,
// 					   size_t idx,
// 					   boost::numeric::ublas::vector< double >& h,
// 					   boost::numeric::ublas::matrix< double >& dh,
// 					   boost::numeric::ublas::matrix< double >& ddh
// 					   );

    void setInEqualityIntegralConstraintPoint (
        const boost::numeric::ublas::vector< double >& x,
        size_t idx,
        boost::numeric::ublas::vector< double >& h,
        boost::numeric::ublas::matrix< double >& dh,
        boost::numeric::ublas::matrix< double >& ddh,
        int pIdx,
        int hBase
    );

    void setHingeConstraintPointY (
        const boost::numeric::ublas::vector< double >& x,
        size_t idx,
        boost::numeric::ublas::vector< double >& h,
        boost::numeric::ublas::matrix< double >& dh,
        boost::numeric::ublas::matrix< double >& ddh,
        int pIdx,
        int hBase
    );

    void setInEqualityNoUpwardsEtaConstraint (
        const boost::numeric::ublas::vector< double >& x,
        size_t idx,
        boost::numeric::ublas::vector< double >& h,
        boost::numeric::ublas::matrix< double >& dh,
        boost::numeric::ublas::matrix< double >& ddh,
        int hBase
    );

    void inEqualityConstraints ( const boost::numeric::ublas::vector<double>& x,
                                 size_t idx,
                                 boost::numeric::ublas::vector<double>& h,
                                 boost::numeric::ublas::matrix<double>& dh,
                                 boost::numeric::ublas::matrix<double>& ddh );

    int getN ( void ) const;

// 	int getM(void) const;

// 	double get_h(void) const;

    double f ( const boost::numeric::ublas::vector< double >& x );

    double f_elastic ( const boost::numeric::ublas::vector< double >& x );

    void df ( boost::numeric::ublas::vector<double> &res, const boost::numeric::ublas::vector<double>& x );

    void ddf_banded2 ( boost::numeric::ublas::matrix<double> &res, const boost::numeric::ublas::vector<double>& x );

    void solve ( boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector<double> &U, boost::numeric::ublas::vector<double> &V );





//     rw::math::Transform3D<> get_planeTbeam(void) const;
//     double get_yTCP(void) const;
//     double get_thetaTCP(void) const;
//     double get_uxTCPy(void) const;
//     double get_uyTCPy(void) const;




private:
    boost::numeric::ublas::vector<double> 	_a;
    boost::numeric::ublas::vector<double> 	_da;


};
/*@}*/
}
}

#endif // MODRUSSELBEAM_HPP
