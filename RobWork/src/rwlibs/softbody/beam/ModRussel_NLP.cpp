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


#include "ModRussel_NLP.hpp"

#include "rwlibs/softbody/beam/RusselIntegrand.hpp"
#include "ModRusselBeamBase.hpp"
#include "rwlibs/softbody/numerics/FdUtil.hpp"
#include "rwlibs/softbody/numerics/TrapMethod.hpp"
#include <rw/math/Constants.hpp>

using namespace Ipopt;
using namespace rwlibs::softbody;




ModRussel_NLP::ModRussel_NLP (
    boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
    boost::shared_ptr< BeamObstaclePlane > obstaclePtr,
    rw::math::Transform3D<> planeTbeam,
    const std::vector<int> & integralIndices
)
    :
    _geomPtr ( geomPtr ),
    _obstaclePtr ( obstaclePtr ),
    _planeTbeam ( planeTbeam ), 
    _integralIndices( integralIndices ),
    _Ee(0.0)
    {
    const int M = getGeometry()->getM();
    
    _a.resize ( M );
    _da.resize ( M );
    
    // angle at x=0 is always assumed zero, so solution vector is of size M-1
    _x.resize( M - 1 );
  
    // default starting guess is zero
    _xinit.resize( M - 1);
    _xinit.clear();
}

ModRussel_NLP::~ModRussel_NLP() {
}



bool ModRussel_NLP::get_nlp_info ( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style ) {
    // number of variables
    n = getGeometry()->getM() - 1;

    // total number of constraints
    m = _integralIndices.size(); 
    // m = _integralIndices.size() + NnoUpwardsConstraints + Nhingeconstraints; 

    // number of nonzero entries in the Jacobian.
    nnz_jac_g = m * n;

    // number of nonzero entries in the Hessian.
    nnz_h_lag = n * n;

    // use the C style indexing (0-based) for sparse matrix indices
    index_style = TNLP::C_STYLE;

    return true;
}



bool ModRussel_NLP::get_bounds_info ( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    const double yTCP = getObstacle()->get_yTCP ( planeTbeam );

    RW_ASSERT ( n == getGeometry()->getM() - 1 );

    // lower bounds of variables
    for ( Index i=0; i<n; i++ ) {
        x_l[i] = -rw::math::Pi;
    }

    // upper bounds of variables
    for ( Index i=0; i<n; i++ ) {
        x_u[i] = rw::math::Pi;
    }

    for (int i = 0; i < m; i++) {        
        g_l[i] = -yTCP;
        g_u[i] = 2.0e19; // magic number for IPOPT indicating no upper bound
        
//         std::cout << "g_l[i]: " << g_l[i] << std::endl;
    }
    
    

    return true;
}



bool ModRussel_NLP::get_starting_point ( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda ) {
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    RW_ASSERT ( init_x == true );
    RW_ASSERT ( init_z == false );
    RW_ASSERT ( init_lambda == false );

    // initialize to the given starting point
    for ( int i = 0; i < n; i++ )
        x[i] = _xinit[i];
    
    return true;
}



bool ModRussel_NLP::eval_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value ) {
    const int M = getGeometry()->getM();
    const double h = getGeometry()->get_h();

    RW_ASSERT ( ( int ) _a.size() == M );
    RW_ASSERT ( n == M - 1 );

    _a[0] = 0.0;
    for ( int i = 1; i < M; i++ )
        _a[i] = x[i-1];

    FdUtil::vectorDerivative ( _a, _da, h ) ;

    RusselIntegrand intgr ( *getGeometry(), _a, _da );
    obj_value = TrapMethod::trapezMethod<RusselIntegrand> ( intgr, M, h );

    return true;
}



// for calculating the total elastic potential energy, given a vector of angles for the beam
bool ModRussel_NLP::eval_f_elastic ( Index n, const Number* x, Number& obj_value ) {
    const int M = getGeometry()->getM();
    const double h = getGeometry()->get_h();

    RW_ASSERT ( ( int ) _a.size() == M );
    RW_ASSERT ( n == M - 1 );

    _a[0] = 0.0;
    for ( int i = 1; i < M; i++ )
        _a[i] = x[i-1];

    FdUtil::vectorDerivative ( _a, _da, h ) ;

    RusselIntegrandEonly intgr ( *getGeometry(), _a, _da );
    obj_value = TrapMethod::trapezMethod<RusselIntegrandEonly> ( intgr, M, h );

    return true;
}




bool ModRussel_NLP::eval_grad_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f ) {
    const double eps = 1.0e-6; // some small value suited for finite-difference
    double xt[n];

    // first make an unpertubed copy of the variables
    for ( int i = 0; i < n; i++ )
        xt[i] = x[i];

    for ( int i = 0; i< n; i++ ) {
        xt[i] = x[i]-eps; // make small negative pertubation for the i'th variable
        double d1;
        eval_f ( n, xt, new_x, d1 ); // evaluate function value for negative pertubation of the i'th variable

        xt[i] = x[i] +eps; // make small positive pertubation for the i'th variable
        double d2;
        eval_f ( n, xt, new_x, d2 );  // evaluate function value for positive pertubation of the i'th variable

        grad_f[i] = ( d2-d1 ) / ( 2*eps ); // centered finite-difference calculation of the first partial derivative with respect to the i'th variable

        xt[i] = x[i]; // restore the i'th variable
    }

    return true;
}



bool ModRussel_NLP::eval_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g ) {
    int gBase = 0; // the base offset into the vector of constraints passed to IPOPT
    
    // go through the list of indices into the beam at which to set a no-penetration constraint value
    for (int i = 0; i < (int) _integralIndices.size(); i++) {
        int pIdx = _integralIndices[i];
        
        // set the gBase'th constraint
        eval_g_point(pIdx, gBase++, n, x, new_x, m, g);    
    }

    return true;
}



bool ModRussel_NLP::eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
    int gBase = 0;  // the base offset into the vector of constraints passed to IPOPT
    
    // go through the list of indices into the beam at which to set a no-penetration constraint derivative value
    for (int i = 0; i < (int) _integralIndices.size(); i++) {
        int pIdx = _integralIndices[i];
        
        // set the gBase'th row in the constraint jacobian
        eval_jac_g_point(pIdx, gBase++, n, x, new_x, m, nele_jac, iRow, jCol, values);
    }

    return true;
}




// evaluates the constraint value of the point at index pIdx on the beam, placing it in the vector g[gBase]
void ModRussel_NLP::eval_g_point ( int pIdx, int gBase, Index n, const Number* x, bool new_x, Index m, Number* g ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2
    const double dy = 7.0;
    const double dx = 0.0;

    // U component
    const double f0U = cos ( 0.0 );
    const double fLU = cos ( x[pIdx] );

    double sumU = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumU += cos ( x[i] ); 
    }
//     double resU = ( ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU)     +          0.5 * 7.0 * sin( x[pIdx] ); // HACK beam thickness hardcoded
//     double resU = ( ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU)     +          (0.5 * dx * cos( x[pIdx] ) - 0.5 *dy * sin(x[pIdx])); 
    double resU = ( ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU);



    // V component
    const double f0V = sin ( 0.0 );
    const double fLV = sin ( x[pIdx] );

    double sumV = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumV += sin ( x[i] ); // TODO the thickness of the beam requires something to be added
    }
//     double resV = ( ( hx / 2.0 ) * ( f0V + fLV ) + hx * sumV )     -          0.5 * 7.0 * cos( x[pIdx] ); // HACK beam thickness hardcoded
    double resV = ( ( hx / 2.0 ) * ( f0V + fLV ) + hx * sumV )     +          (-0.5 * dy * cos( x[pIdx] ) - 0.5*dx*sin(x[pIdx]) ); // HACK beam thickness hardcoded
//     double resV = ( ( hx / 2.0 ) * ( f0V + fLV ) + hx * sumV );

    
    g[gBase] = resU * uxTCPy        + resV * uyTCPy; // require h(x) > 0
}





// evaluates the value of constraint jacobian of the point at index pIdx on the beam, placing it in the constraint jacobian with row gBase
void ModRussel_NLP::eval_jac_g_point ( int pIdx, int gBase, Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values ) {
  const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2
    const double dy = 7.0;
    const double dx = 0.0;
    
    // populate the mxn jacobian of the constraint functions (eg. 1x31 with one integral constraint and 32 slices)
    if ( values == NULL ) {
        // return the structure of the jacobian
        // see http://www.coin-or.org/Ipopt/documentation/node57.html#app.triplet        
        for (int j = 0; j < n; j++) { // loop through all the columns (refering to each x[j]            
            iRow[gBase*n + j] = gBase; // the constraint index (e.g. g[0]) is gBase and is the row in which to place it
            jCol[gBase*n + j] = j; // the x index            
        }
    } 
    else {
        // return the values of the jacobian of the constraints
        for ( int j = 0; j < n; j++ ) {
            if ( pIdx == j ) {
//                 values[gBase*n + j] = 0.5 * hx * uyTCPy * cos ( x[j] ) - 0.5 * hx * uxTCPy * sin ( x[j] );
                // HACK beam thickness hardcoded
                values[gBase*n + j] = 
                (  (-0.5 * hx * sin (x[j]) )  ) * uxTCPy +        // U part
                (  (0.5 * hx * cos (x[j]) ) +  (-0.5*dx*cos(x[j]) + 0.5*dy*sin(x[j]))  ) * uyTCPy;

                
                //(  (-0.5 * hx * sin (x[j]) )  +  (-0.5*dy*cos(x[j]) - 0.5*dx*sin(x[j]))   ) * uxTCPy +        // U part
            } 
            else if ( j < pIdx ) {
                values[gBase*n + j] = hx * uyTCPy * cos ( x[j] ) - hx * uxTCPy * sin ( x[j] );
            } 
            else
                values[gBase*n + j] = 0.0;
        }
    }
}



// evaluate the problem hessian 
//
// currently we just let IPOPT calculate this numerically by having set the options to ( "hessian_approximation", "limited-memory" )
bool ModRussel_NLP::eval_h ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
    RW_ASSERT(false);
    if ( values == NULL ) {
        // return the structure. 
    } else {
        // return the values. 
    }

    return true;
}



void ModRussel_NLP::finalize_solution ( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq ) {
    for (int i = 0; i < n; i++) 
        _x[i] = x[i];
/*    
    std::cout << "start constraints\n";
    for (int i = 0; i < m; i++) {
            std::cout << g[i] << std::endl;
    }
    std::cout << "end constraints\n";
    */
    // calculate the elastic energy
    eval_f_elastic(n, x, _Ee);
}



boost::shared_ptr< BeamGeometry > ModRussel_NLP::getGeometry ( void ) const {
    return _geomPtr;
}



boost::shared_ptr< BeamObstaclePlane > ModRussel_NLP::getObstacle ( void ) const {
    return _obstaclePtr;
}



rw::math::Transform3D< double > ModRussel_NLP::get_planeTbeam ( void ) const {
    return _planeTbeam;
}



const boost::numeric::ublas::vector< double >& ModRussel_NLP::getSolution ( void ) const {
    return _x;
}



double ModRussel_NLP::getEnergyElastic ( void ) const {
    return _Ee;
}



void ModRussel_NLP::setStartingGuess ( const boost::numeric::ublas::vector< double >& xinituser ) {
    RW_ASSERT(xinituser.size() == _xinit.size() + 1);
    
    for (int i = 0; i < (int) _xinit.size(); i++)
        _xinit[i] = xinituser[i+1];
}
