/*
    Copyright 2013 <copyright holder> <email>

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

#include <cassert>
#include <iostream>

#include "rwlibs/softbody/beam/RusselIntegrand.hpp"
#include "ModRusselBeamBase.hpp"
#include "rwlibs/softbody/numerics/FdUtil.hpp"
#include "rwlibs/softbody/numerics/TrapMethod.hpp"
#include <rw/math/Constants.hpp>

using namespace Ipopt;

using namespace rwlibs::softbody;

ModRussel_NLP::ModRussel_NLP (
    boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
    boost::shared_ptr< BeamObstaclePlane > obstaclePtr      ,
    rw::math::Transform3D<> planeTbeam,
    const std::vector<int> & integralIndices
)
    :
    _geomPtr ( geomPtr ),
    _obstaclePtr ( obstaclePtr ),
    _planeTbeam ( planeTbeam ), 
    _integralIndices( integralIndices )
    {
    const int M = getGeometry()->getM();
    _a.resize ( M );
    _da.resize ( M );
    _x.resize( M - 1 );
    _xinit.resize( M - 1);
    
    // default starting guess is zero
    _xinit.clear();
    
}

ModRussel_NLP::~ModRussel_NLP() {

}



bool ModRussel_NLP::get_nlp_info ( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style ) {
    // number of variables
    n = getGeometry()->getM() - 1;

    // total number of constraints
    m = _integralIndices.size();

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
//     RW_ASSERT ( m == 1 );

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
        g_u[i] = 2.0e19; // no upper bound
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
    //const int N = getN();
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



bool ModRussel_NLP::eval_grad_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f ) {
    const double eps = 1.0e-6;
    double xt[n];

    // aanoying to copy manually!
    for ( int i = 0; i < n; i++ )
        xt[i] = x[i];

    for ( int i = 0; i< n; i++ ) {
        xt[i] = x[i]-eps;
        double d1;
        eval_f ( n, xt, new_x, d1 );

        xt[i] = x[i] +eps;
        double d2;
        eval_f ( n, xt, new_x, d2 );

        grad_f[i] = ( d2-d1 ) / ( 2*eps );

        xt[i] = x[i]; // making sure to copy back original value
    }

    return true;
}



bool ModRussel_NLP::eval_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g ) {
//     RW_ASSERT ( m == 1 );

//     int pIdx = n - 1;
    int gBase = 0;
    for (int i = 0; i < (int) _integralIndices.size(); i++) {
        int pIdx = _integralIndices[i];
        eval_g_point(pIdx, gBase++, n, x, new_x, m, g);    
    }
    
    


    return true;
}



bool ModRussel_NLP::eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
//     RW_ASSERT ( m == 1 );
    
//     int pIdx = n - 1; // TODO make according to idxList
//     int gBase = 0;
    
//     eval_jac_g_point(pIdx, gBase++, n, x, new_x, m, nele_jac, iRow, jCol, values);
    
    int gBase = 0;
    for (int i = 0; i < (int) _integralIndices.size(); i++) {
        int pIdx = _integralIndices[i];
//         eval_g_point(pIdx, gBase++, n, x, new_x, m, g);    
        eval_jac_g_point(pIdx, gBase++, n, x, new_x, m, nele_jac, iRow, jCol, values);
    }

    return true;
}





void ModRussel_NLP::eval_g_point ( int pIdx, int gBase, Index n, const Number* x, bool new_x, Index m, Number* g ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2

    // U component
    const double f0U = cos ( 0.0 );
    const double fLU = cos ( x[pIdx] );

    double sumU = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumU += cos ( x[i] );
    }
    double resU = ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU;



    // V component
    const double f0V = sin ( 0.0 );
    const double fLV = sin ( x[pIdx] );

    double sumV = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumV += sin ( x[i] );
    }
    double resV = ( hx / 2.0 ) * ( f0V + fLV ) + hx * sumV;

    
    g[gBase] = resU * uxTCPy        + resV * uyTCPy; // require h(x) > 0
}





void ModRussel_NLP::eval_jac_g_point ( int pIdx, int gBase, Index n, const Number* x, bool new_x, Index m, Index nele_jac, Index* iRow, Index* jCol, Number* values ) {
  const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2
    
    
    // populate the mxn jacobian of the constraint functions (eg. 1x31 with one integral constraint and 32 slices)
    
    if ( values == NULL ) {
        // return the structure of the jacobian
        // see http://www.coin-or.org/Ipopt/documentation/node57.html#app.triplet
        // TODO include sparsity
        
        // gBase - index of the point to constrain
        
        for (int j = 0; j < n; j++) { // loop through all the columns (refering to each x[j]            
            iRow[gBase*n + j] = gBase; // the constraint index (e.g. g[0]) is gBase and is the row in which to place it
            jCol[gBase*n + j] = j; // the x index            
        }
    } 
    else {
        // return the values of the jacobian of the constraints
        for ( int j = 0; j < n; j++ ) {
            if ( pIdx == j ) {
                values[gBase*n + j] = 0.5 * hx * uyTCPy * cos ( x[j] ) - 0.5 * hx * uxTCPy * sin ( x[j] );
            } 
            else if ( j < pIdx ) {
                values[gBase*n + j] = hx * uyTCPy * cos ( x[j] ) - hx * uxTCPy * sin ( x[j] );
            } 
            else
                values[gBase*n + j] = 0.0;
        }
    }
}



bool ModRussel_NLP::eval_h ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
    RW_ASSERT(false);
    if ( values == NULL ) {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.

        // the hessian for this problem is actually dense
//         Index idx=0;
//         for ( Index row = 0; row < 4; row++ ) {
//             for ( Index col = 0; col <= row; col++ ) {
//                 iRow[idx] = row;
//                 jCol[idx] = col;
//                 idx++;
//             }
//         }
//
//         assert ( idx == nele_hess );
    } else {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only

        // fill the objective portion
//         values[0] = obj_factor * ( 2*x[3] ); // 0,0
//
//         values[1] = obj_factor * ( x[3] ); // 1,0
//         values[2] = 0.;                    // 1,1
//
//         values[3] = obj_factor * ( x[3] ); // 2,0
//         values[4] = 0.;                    // 2,1
//         values[5] = 0.;                    // 2,2
//
//         values[6] = obj_factor * ( 2*x[0] + x[1] + x[2] ); // 3,0
//         values[7] = obj_factor * ( x[0] );               // 3,1
//         values[8] = obj_factor * ( x[0] );               // 3,2
//         values[9] = 0.;                                  // 3,3
//
//
//         // add the portion for the first constraint
//         values[1] += lambda[0] * ( x[2] * x[3] ); // 1,0
//
//         values[3] += lambda[0] * ( x[1] * x[3] ); // 2,0
//         values[4] += lambda[0] * ( x[0] * x[3] ); // 2,1
//
//         values[6] += lambda[0] * ( x[1] * x[2] ); // 3,0
//         values[7] += lambda[0] * ( x[0] * x[2] ); // 3,1
//         values[8] += lambda[0] * ( x[0] * x[1] ); // 3,2
//
//         // add the portion for the second constraint
//         values[0] += lambda[1] * 2; // 0,0
//
//         values[2] += lambda[1] * 2; // 1,1
//
//         values[5] += lambda[1] * 2; // 2,2
//
//         values[9] += lambda[1] * 2; // 3,3
    }

    return true;
}

void ModRussel_NLP::finalize_solution ( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq ) {
    for (int i = 0; i < n; i++) 
        _x[i] = x[i];

    // For this example, we write the solution to the console
//     std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
//     for ( Index i=0; i<n; i++ ) {
//         std::cout << "x[" << i << "] = " << x[i] << std::endl;
//     }

//     std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
//     for ( Index i=0; i<n; i++ ) {
//         std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
//     }
//     for ( Index i=0; i<n; i++ ) {
//         std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
//     }

    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;

    std::cout << std::endl << "Final value of the constraints:" << std::endl;
    for ( Index i=0; i<m ; i++ ) {
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
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

void ModRussel_NLP::setStartingGuess ( const boost::numeric::ublas::vector< double >& xinituser ) {
    RW_ASSERT(xinituser.size() == _xinit.size() + 1);
    
    for (int i = 0; i < (int) _xinit.size(); i++)
        _xinit[i] = xinituser[i+1];
}
