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






#include <rw/common/macros.hpp>

#include "ModRusselBeamIpopt.hpp"
#include "ModRussel_NLP.hpp"

using namespace Ipopt;
using namespace rwlibs::softbody;


ModRusselBeamIpopt::ModRusselBeamIpopt (
    boost::shared_ptr< BeamGeometry > geomPtr,
    boost::shared_ptr< BeamObstaclePlane > obstaclePtr,
    int M
) :
    ModRusselBeamBase ( geomPtr, obstaclePtr, M ) {

}



ModRusselBeamIpopt::~ModRusselBeamIpopt() {

}



void ModRusselBeamIpopt::solve ( boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector< double >& U, boost::numeric::ublas::vector< double >& V ) {
    computeIntegralIndicies();
    
    _nlp = new ModRussel_NLP( getGeometry(), getObstacle(), get_planeTbeam(), getIntegralIndices() ); 

    _app = IpoptApplicationFactory();

    ApplicationReturnStatus status;
    status = _app->Initialize();
    if ( status != Solve_Succeeded ) {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        RW_THROW("Error during initialization");
    }

    _app->Options()->SetNumericValue ( "tol", getAccuracy() );
    _app->Options()->SetNumericValue ( "max_cpu_time", 10.0 );
    _app->Options()->SetIntegerValue ( "print_level", 4 );
    _app->Options()->SetStringValue ( "mu_strategy", "adaptive" );

    _app->Options()->SetStringValue ( "hessian_approximation", "limited-memory" );
//     _app->Options()->SetStringValue ( "derivative_test", "first-order" );

    status = _app->OptimizeTNLP ( _nlp );
    if ( status == Solve_Succeeded ) {
        std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
    } 
    else {
        std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    }
    
    ModRussel_NLP *nlp = static_cast<ModRussel_NLP *> ( GetRawPtr<TNLP>(_nlp) );
    boost::numeric::ublas::vector< double > res = nlp->getSolution();
    int N =  res.size();
    
    integrateAngleU ( U, res );
    integrateAngleV ( V, res );

    xinituser[0] = 0.0;
    for ( int i = 0; i < N; i++ )
        xinituser[i+1] = res[i];
}

