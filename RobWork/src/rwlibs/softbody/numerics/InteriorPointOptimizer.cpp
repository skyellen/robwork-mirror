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

/*
 * InteriorPointOptimizer.cpp
 *
 *  Created on: Jan 15, 2009
 *      Author: lpe
 */

#include "InteriorPointOptimizer.hpp"

using namespace boost::numeric::ublas;
using namespace rw::math;
using namespace rwlibs::softbody;


namespace internal {
void compute_f_info_EXT ( const vector<double>& x, double &f, boost::numeric::ublas::vector<double> &df, boost::numeric::ublas::matrix<double> &ddf ) {

}

void compute_con_info_i_EXT ( int i,
                              const vector<double>& x,
                              boost::numeric::ublas::vector<double> &a,
                              boost::numeric::ublas::matrix<double> &da,
                              boost::numeric::ublas::matrix<double> &dda ) {

}

}

void InteriorPointOptimizer::initialize() {
    _x = vector<double> ( N );
    _s = vector<double> ( M );
    _z = vector<double> ( M );
    _df = vector<double> ( N );
    _ddf = matrix<double> ( N,N );
    _a = vector<double> ( M );
    _da = matrix<double> ( M,N );
    _dda = matrix<double> ( N,N );
    _accuracy = 1e-6;
}

InteriorPointOptimizer::InteriorPointOptimizer ( size_t n,
        size_t m,
        ObjectFunction objectFunction,
        ConstraintFunction constraintFunction ) :
    compute_f_info_EXT ( objectFunction ),
    compute_con_info_i_EXT ( constraintFunction ),
    N ( n ),
    M ( m ) {
    // compute_f_info_EXT = boost::bind(&InteriorPointOptimizer::objectFunction, this, _1, _2, _3, _4);
    // compute_con_info_i_EXT = boost::bind(&InteriorPointOptimizer::constraintFunction, this, _1, _2, _3, _4, _5);
    initialize();
}


InteriorPointOptimizer::InteriorPointOptimizer ( size_t n, size_t m ) :
    compute_f_info_EXT ( boost::bind ( &InteriorPointOptimizer::objectFunction, this, _1, _2, _3, _4 ) ),
    compute_con_info_i_EXT ( boost::bind ( &InteriorPointOptimizer::constraintFunction, this, _1, _2, _3, _4, _5 ) ),
    N ( n ),
    M ( m ) {
    initialize();
}


InteriorPointOptimizer::~InteriorPointOptimizer() {

}

void InteriorPointOptimizer::objectFunction ( const boost::numeric::ublas::vector<double>& x,
        double &f,
        boost::numeric::ublas::vector<double> &df,
        boost::numeric::ublas::matrix<double> &ddf ) {
    RW_THROW ( "Object function undefined for InteriorPointMethod" );
}

void InteriorPointOptimizer::constraintFunction ( const boost::numeric::ublas::vector<double>& x,
        int i,
        boost::numeric::ublas::vector<double> &a,
        boost::numeric::ublas::matrix<double> &da,
        boost::numeric::ublas::matrix<double> &dda ) {
    RW_THROW ( "Constraint function undefined for InteriorPointMethod" );
}



void InteriorPointOptimizer::setAccuracy ( double accuracy ) {
    _accuracy = accuracy;
}

double InteriorPointOptimizer::getAccuracy() {
    return _accuracy;
}



// Verification of objective and constraints (needs an admissible x)
void InteriorPointOptimizer::verify_user_defined_objective_and_constraints() {
    double h,delta,ftest,fold,atest;
    vector<double> xtest ( N ),dxtest ( N );
    matrix<double> ddfold ( N,N );
    vector<double> dfold ( N );
    matrix<double> ddaold ( N,N );
    matrix<double> daold ( M,N );
    vector<double> aold ( M );
    /*
        xtest.clear();
        dxtest.clear();
        ddfold.clear();
        dfold.clear();
        ddaold.clear();
        daold.clear();
        aold.clear();*/

// 	std::cout<<"X = "<<_x<<std::endl;
    for ( size_t j=0; j<M; j++ ) {
        compute_con_info_i_EXT ( _x, j, _a, _da, _dda );
        std::cout << "Constraint " << j << " has value " << _a ( j ) <<std::endl;
        if ( _a ( j ) <=0 )
            RW_THROW ( "Initial value: Violation of constraint!" );


    }

    return;

    delta=0.0;
    h=0.00001;

    // TODO remove seed
//     unsigned seed = 42;
//     srand(seed);

    for ( size_t i=0; i<N; i++ ) {
        xtest ( i ) = delta*rand() /32768.0;
        dxtest ( i ) = h*rand() /32768.0;
    }
    _x += xtest;
    compute_f_info_EXT ( _x, fold, dfold, ddfold );
    _x += dxtest;
    compute_f_info_EXT ( _x, _f, _df, _ddf );
    ftest = fold+inner_prod ( dfold,dxtest );
    std::cout << "Objective. " << "First order:" << "h: "<< h<< " Rel. error:"<< ( ftest-_f ) / ( _f-fold ) <<std::endl;
    for ( size_t jj = 0; jj < N; ++jj )
        for ( size_t kk = 0; kk < N; ++kk )
            ftest+=0.5*_ddf ( jj,kk ) *dxtest ( jj ) *dxtest ( kk );
    std::cout<< "Objective. " << "Second order:" << "h: "<< h<< " Rel. error:"<< ( ftest-_f ) / ( _f-fold ) <<std::endl;
    _x -= dxtest;

    for ( size_t j=0; j<M; j++ ) {
        compute_con_info_i_EXT ( _x, j, aold, daold, ddaold );
        _x += dxtest;
        compute_con_info_i_EXT ( _x, j, _a, _da, _dda );
        atest=aold ( j );
        for ( size_t i=0; i<N; i++ )
            atest += daold ( j,i ) *dxtest ( i );
        std::cout << "Constraint no. " << j <<". First order:" << "h: "<< h<< " Rel. error:"<< ( atest-_a ( j ) ) / ( _a ( j )-aold ( j ) ) <<std::endl;
        for ( size_t jj = 0; jj < N; ++jj )
            for ( size_t kk = 0; kk < N; ++kk )
                atest += 0.5*_dda ( jj,kk ) *dxtest ( jj ) *dxtest ( kk );
        std::cout << "Constraint no. " << j <<". Second order:" << "h: "<< h<< " Rel. error:"<< ( atest-_a ( j ) ) / ( _a ( j )-aold ( j ) ) <<std::endl;
        _x -= dxtest;
    }
}



void InteriorPointOptimizer::choleskySolve ( int n_e, int bw, matrix<double> &A, vector<double> &b, vector<double> &x ) {

    // "n_e" is the dimension of the system. "bw" is the bandwidth measured as the max-distance
    // from the diagonal of a non-zero element in A.
    vector<double> y ( n_e );
    int ist,jst;

    for ( int k = 0; k < n_e; ++k ) {
        y ( k ) = b ( k );
        jst=k-bw;
        if ( jst<0 ) jst=0;
        for ( int jj = jst; jj < k; ++jj ) {
            A ( k,k )-=A ( k,jj ) *A ( k,jj );
            y ( k )-=A ( k,jj ) *y ( jj );
        }
        A ( k,k ) =sqrt ( A ( k,k ) );
        y ( k ) /=A ( k,k );
        ist=k+bw+1;
        if ( ist>n_e ) ist=n_e;
        for ( int ii = k+1; ii < ist; ++ii ) {
            jst=k-bw;
            if ( jst<0 ) jst=0;
            for ( int jj = jst; jj < k; ++jj )
                A ( ii,k )-=A ( ii,jj ) *A ( k,jj );
            A ( ii,k ) =A ( ii,k ) /A ( k,k );
        }
    }

    for ( int k = n_e-1; k >=0; --k ) {
        x ( k ) =y ( k );
        ist=k+bw+1;
        if ( ist>n_e ) ist=n_e;
        for ( int ii = k+1; ii < ist; ++ii )
            x ( k )-=A ( ii,k ) *x ( ii );
        x ( k ) /=A ( k,k );
    }
}


// Computation of objective contributions to the linear equations
void InteriorPointOptimizer::compute_f_info ( matrix<double> &A, vector<double> &RHS ) {

    compute_f_info_EXT ( _x,  _f, _df, _ddf );
    for ( size_t jj = 0; jj < N; ++jj ) {
        for ( size_t kk = 0; kk < N; ++kk )
            A ( jj, kk ) += _ddf ( jj, kk );
        RHS ( jj ) -= _df ( jj );
    }

}


// Computation of constraint contributions to the linear equations
void InteriorPointOptimizer::compute_con_info ( matrix<double> &A, vector<double> &RHS ) {

    double zs;

    for ( size_t i = 0; i < M; ++i ) {
        compute_con_info_i_EXT ( _x, i, _a, _da, _dda );
        zs = _z ( i ) /_s ( i );
        for ( size_t jj = 0; jj < N; ++jj ) {
            for ( size_t kk = 0; kk < N; ++kk )
                A ( jj,kk ) += -_z ( i ) *_dda ( jj,kk ) +zs*_da ( i,jj ) *_da ( i,kk );
            RHS ( jj ) += ( _mu/_s ( i )-zs*_a ( i ) +_z ( i ) ) *_da ( i,jj );
        }
    }

}

// merit_info and differentiated merit_info implements the merit function given by Eq. 19.26 in Nocedal and Wright
void InteriorPointOptimizer::merit_info ( vector<double> &x, vector<double> &s, double &phi, double &eta ) {
    phi = 0;
    phi +=_f;
    for ( size_t i = 0; i < M; ++i ) {
        if ( _a ( i ) >s ( i ) )
            phi+=eta* ( _a ( i )-s ( i ) )-_mu*log ( s ( i ) );
        else
            phi+=eta* ( s ( i )-_a ( i ) )-_mu*log ( s ( i ) );
    }
}

void InteriorPointOptimizer::Dmerit_info ( vector<double> &x, vector<double> &s, vector<double> &dx, vector<double> &ds, double &Dphi, double &eta ) {

    Dphi=0;
    for ( size_t i = 0; i < N; ++i )
        Dphi += _df ( i ) *dx ( i );

    for ( size_t j = 0; j < M; ++j ) {
        Dphi-= _mu*ds ( j ) /s ( j );
        if ( _a ( j ) >s ( j ) ) {
            Dphi-=eta*ds ( j );
            for ( size_t i = 0; i < N; ++i )
                Dphi += eta*_da ( j,i ) *dx ( i );
        } else {
            Dphi+=eta*ds ( j );
            for ( size_t i = 0; i < N; ++i )
                Dphi -= eta*_da ( j,i ) *dx ( i );
        }

    }
}



// trust region based update. Method used is Nocedal and Wright p. 572-577. NOTICS: Adaptive updates of eta for efficiency is still not implemented
void InteriorPointOptimizer::update ( vector<double> &x, vector<double> &dx, vector<double> &s, vector<double> &z ) {
    double phi0,phi,Dphi;
    vector<double> x0 ( N );
    vector<double> s0 ( M ),ds ( M );
    double tmax,t;
    double fval,eps;
    //double oldfval;
    //double oldt;
    size_t i,jj;


    _eta=1;
    eps=0.005;

    // Controlling not to pass constraint boundaries
    tmax=1;
    for ( i = 0; i < M; ++i ) {
        ds ( i ) = _a ( i )-s ( i );
        for ( jj = 0; jj < N; ++jj )
            ds ( i ) += _da ( i,jj ) *dx ( jj );
        if ( ( -ds ( i ) *tmax ) >s ( i ) ) tmax=-s ( i ) /ds ( i );
    }
    tmax*= ( 1-eps );

    // Merit function is Formula 19.26 in [1]. We use the merit function for a trust region.
    merit_info ( x,s,phi0,_eta );
    Dmerit_info ( x,s,dx,ds,Dphi,_eta );

    x0=x;
    s0=s;
    t=tmax;
    x=x0+t*dx;
    s=s0+t*ds;
    compute_f_info_EXT ( x, _f, _df, _ddf );
    for ( size_t j = 0; j < M; ++j )
        compute_con_info_i_EXT ( x, j, _a, _da, _dda );
    merit_info ( x,s,phi,_eta );
    fval= ( phi-phi0-0.8*t*Dphi ) / ( phi0-phi );

//     std::cout << "update 1 start\n";
    if ( fval>0 ) {
        while ( fval>0 ) {
//             std::cout << "t: " << t << std::endl;
//             std::cout << "fval: " << fval << std::endl;

            //oldt=t;
            //oldfval=fval;
            t*=0.8;
            x=x0+t*dx;
            s=s0+t*ds;
            compute_f_info_EXT ( x, _f, _df, _ddf );
            for ( size_t j = 0; j < M; ++j )
                compute_con_info_i_EXT ( x, j, _a, _da, _dda );
            merit_info ( x,s,phi,_eta );
            fval= ( phi-phi0-0.8*t*Dphi ) / ( phi0-phi );
        }
    }
//     std::cout << "fval on loop termination: " << fval << std::endl;
//     std::cout << "update 1 end\n";

    if ( t>tmax ) {
        std::cout << "t>tmax" << t<<tmax<<std::endl;
        exit ( 1 );
    }
    for ( size_t j = 0; j < M; ++j )
        z ( j ) =_mu/s ( j );


}


vector<double> InteriorPointOptimizer::solve ( const vector<double>& x_init ) {
    _x = x_init;
    matrix<double> A ( N, N );
    vector<double> RHS ( N );
    vector<double> dx ( N );
    zero_matrix<double> A0 ( N, N );
    zero_vector<double> RHS0 ( N );
    vector<double> oldx ( N ),diffx ( N );
    int n_eq_solves=0;
    //int bw=N;
    double testval;
    //_mu=0.01;
    _mu=0.01;



    // user defined initialization of primal variables
    //initialize_EXT();

    // initialization of dual variables
    for ( size_t k = 0; k < M; ++k ) {
        compute_con_info_i_EXT ( _x, k, _a, _da, _dda );
        _s ( k ) = _a ( k );
        _z ( k ) = 0;
    }

    // verification of user defined routines
    verify_user_defined_objective_and_constraints();

    const int MAXITERATIONS = 30;

//     assert(false);
    // iterations for solving the optimization problem
    double diff=1;

    while ( diff> ( _accuracy*_accuracy ) && _mu>_accuracy ) {
        //std::cout<<std::endl<<std::endl;

        oldx = _x;
        testval=1;

        double iterations = 0;
        while ( testval>_mu*_mu ) {


            //		std::cout<<std::endl<<std::endl;
            A = A0;
            RHS = RHS0;
//             std::cout << "1 start\n";
            compute_f_info ( A, RHS );
//             std::cout << "1 end\n";

//             std::cout << "2 start\n";
            compute_con_info ( A, RHS );
//             std::cout << "2 end\n";
            //std::cout<<"A = "<<A<<std::endl;
            //std::cout<<"RHS = "<<RHS<<std::endl;
            //choleskySolve(N,bw,A,RHS,dx);
            //std::cout<<"Det = "<<LinearAlgebra::det(A)<<std::endl;


            
//             if (fabs(LinearAlgebra::det(A))< 1.0e-6) {
//             std::cout<<"A = "<<A<<std::endl;
//             std::cout<<std::endl;
//             std::cout<<"DDF = "<<_ddf<<std::endl;
//             std::cout<<"DF = "<<_df<<std::endl;
//             }
            

//             std::cout << "3 start\n";
            dx = prod ( LinearAlgebra::inverse ( A ), RHS );
//             std::cout << "3 end\n";
// 			std::cout<<"dx = "<<dx<<std::endl;
            n_eq_solves++;

//             std::cout << "4 start\n";
            update ( _x,dx,_s,_z );
//             std::cout << "4 end\n";
            //std::cout<<"dx = "<<dx<<std::endl;
            double testvalold = testval;
//             std::cout << "5 start\n";
            testval=inner_prod ( dx,dx );
//             std::cout << "5 end\n";
            std::cout<<"TestVal = "<<testval<<" "<<_f<<std::endl;

            double change = fabs ( testval - testvalold ) ;
// 	    std::cout << "change: " << change << std::endl;

//             if ( change < 1.0e-5 ) {
//                 std::cout << "Breaking inner loop because of small change in testval!\n";
//                 break;
//             }
            if ( change > 1.0e10 ) {
                std::cout << "change was " << change << " indicating something is horribly wrong\n";
                RW_THROW ( "too large change in testval!" );
            }
            if ( ++iterations > MAXITERATIONS ) {
                std::cout << "Halting search because of MAXITERATIONS = " << MAXITERATIONS << " reached!\n";
                break;
            }
        }
        diffx = _x-oldx;
//         std::cout << "6 start\n";
        diff=inner_prod ( diffx,diffx );
//         std::cout << "6 end\n";
        // update of mu. NOTICE: Should be made adaptive for efficiency
        _mu*=0.1;
        std::cout << "_mu: " << _mu << std::endl;
    }
//     std::cout <<"Search finished. Result is "<< _x << "\nThe accuracy is at least "<< sqrt(diff) <<". The number of LE solves was "<<n_eq_solves<<std::endl;
    std::cout <<"Search finished. The accuracy is at least "<< sqrt ( diff ) <<". The number of LE solves was "<<n_eq_solves<<std::endl;
    compute_f_info_EXT ( _x,  _f, _df, _ddf );
    std::cout<<"Objective = "<<_f<<std::endl;

    return _x;
}
