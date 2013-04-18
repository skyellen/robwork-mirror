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
    rw::math::Transform3D<> planeTbeam
)
    :
    _geomPtr ( geomPtr ),
    _obstaclePtr ( obstaclePtr ),
    _planeTbeam ( planeTbeam ) {
    const int M = getGeometry()->getM();
    _a.resize ( M );
    _da.resize ( M );
    _x.resize( M - 1 );
}

ModRussel_NLP::~ModRussel_NLP() {

}



bool ModRussel_NLP::get_nlp_info ( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style ) {
    // number of variables
    n = getGeometry()->getM() - 1;

    // total number of constraints
    m = 1;

    // number of nonzero entries in the Jacobian.
    nnz_jac_g = n;


    // number of nonzero entries in the Hessian.
    nnz_h_lag = n * n;

    // use the C style indexing (0-based) for sparse matrix indices
    index_style = TNLP::C_STYLE;

    return true;
}




bool ModRussel_NLP::get_bounds_info ( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
    double yTCP = getObstacle()->get_yTCP ( planeTbeam );

    RW_ASSERT ( n == getGeometry()->getM() - 1 );
    RW_ASSERT ( m == 1 );

    // lower bounds of variables
    for ( Index i=0; i<n; i++ ) {
        x_l[i] = -rw::math::Pi;
    }

    // upper bounds of variables
    for ( Index i=0; i<n; i++ ) {
        x_u[i] = rw::math::Pi;
    }

    g_l[0] = -yTCP;
    
    g_u[0] = 2.0e19; // no upper bound

    /*
    // the first constraint g1 has a lower bound of 25
    g_l[0] = 25;
    // the first constraint g1 has NO upper bound, here we set it to 2e19.
    // Ipopt interprets any number greater than nlp_upper_bound_inf as
    // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
    // is 1e19 and can be changed through ipopt options.
    g_u[0] = 2e19;

    // the second constraint g2 is an equality constraint, so we set the
    // upper and lower bound to the same value
    g_l[1] = g_u[1] = 40.0;
    */

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
        x[i] = 0.0;

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
    RW_ASSERT ( m == 1 );

    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
//     double yTCP = getObstacle()->get_yTCP ( planeTbeam );
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2

    int pIdx = n - 1;

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


//     g[0] = resU * uxTCPy        + resV * uyTCPy        + yTCP; // require h(x) > 0
    g[0] = resU * uxTCPy        + resV * uyTCPy; // require h(x) > 0

    return true;
}



bool ModRussel_NLP::eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
    RW_ASSERT ( m == 1 );
    
    int pIdx = n - 1;
    
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam(); //
//     double yTCP = getObstacle()->get_yTCP ( planeTbeam );
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( n );
    const double uxTCPy =  ModRusselBeamBase::get_uxTCPy ( planeTbeam ); // u2
    const double uyTCPy =  ModRusselBeamBase::get_uyTCPy ( planeTbeam ); // v2
    
    if ( values == NULL ) {
        // return the structure of the jacobian
        // see http://www.coin-or.org/Ipopt/documentation/node57.html#app.triplet
        // TODO include sparsity
        for (int i = 0; i < n; i++) {
                iRow[i] = 0;
                jCol[i] = i;            
        }
    } else {
        // return the values of the jacobian of the constraints
        for ( int i = 0; i < n; i++ ) {
            if ( pIdx == i ) {
                values[i] = 0.5 * hx * uyTCPy * cos ( x[i] ) - 0.5 * hx * uxTCPy * sin ( x[i] );
            } else if ( i < pIdx ) {
                values[i] = hx * uyTCPy * cos ( x[i] ) - hx * uxTCPy * sin ( x[i] );
            } else
                values[i] = 0.0;
        }

//         values[0] = x[1]*x[2]*x[3]; // 0,0
//         values[1] = x[0]*x[2]*x[3]; // 0,1
//         values[2] = x[0]*x[1]*x[3]; // 0,2
//         values[3] = x[0]*x[1]*x[2]; // 0,3
//
//         values[4] = 2*x[0]; // 1,0
//         values[5] = 2*x[1]; // 1,1
//         values[6] = 2*x[2]; // 1,2
//         values[7] = 2*x[3]; // 1,3
    }

    return true;
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
