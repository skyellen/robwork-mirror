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

#include "ModRusselBeam.hpp"

#include "rw/math/Math.hpp"


#include "rwlibs/softbody/numerics/TrapMethod.hpp"
#include "rwlibs/softbody/numerics/FdUtil.hpp"

using namespace std;
using namespace rw::math;
using namespace boost::numeric::ublas;
using namespace rwlibs::softbody;


#include "BeamGeometry.hpp"
#include "rwlibs/softbody/numerics/Interpolation.hpp"

#include "rwlibs/softbody/beam/RusselIntegrand.hpp"

static int NFCALLS;


const char DIVIDER[] = "--------------------------------------------------------------------------------";


ModRusselBeam::ModRusselBeam (
    boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
    boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
    int M
) : 
ModRusselBeamBase(geomPtr, obstaclePtr, M),
_a ( M ), 
_da ( M ) 
{
    RW_ASSERT ( getM() >= 2 );
}














int ModRusselBeam::getN ( void )  const {
    return getM() -1;
}






// total energy integrated for the beam: Int (eg + ee) dV
double ModRusselBeam::f ( const boost::numeric::ublas::vector<double>& x ) {
    //const int N = getN();
    const int M = getM();
    const double h = get_h();

    RW_ASSERT ( ( int ) x.size() == getN() );
    RW_ASSERT ( ( int ) _a.size() == getM() );

    _a[0] = 0.0;
    for ( int i = 1; i < M; i++ )
        _a[i] = x[i-1];

    FdUtil::vectorDerivative ( _a, _da, get_h() ) ;

    RusselIntegrand intgr ( *getGeometry(), _a, _da );
    double val = TrapMethod::trapezMethod<RusselIntegrand> ( intgr, M, h );

    NFCALLS++;

    return val;
}


// total elastic energy integrated for the beam: Int (ee) dV
double ModRusselBeam::f_elastic ( const boost::numeric::ublas::vector< double >& x ) {
    const int M = getM();
    const double h = get_h();

    RW_ASSERT ( ( int ) x.size() == getN() );
    RW_ASSERT ( ( int ) _a.size() == getM() );

    _a[0] = 0.0;
    for ( int i = 1; i < M; i++ )
        _a[i] = x[i-1];

    FdUtil::vectorDerivative ( _a, _da, get_h() ) ;

    RusselIntegrandEonly intgr ( *getGeometry(), _a, _da );
    double val = TrapMethod::trapezMethod<RusselIntegrandEonly> ( intgr, M, h );

    NFCALLS++;

    return val;
}



//Computing gradient for object function
void ModRusselBeam::df ( boost::numeric::ublas::vector<double> &res, const boost::numeric::ublas::vector<double>& x ) {
//     boost::numeric::ublas::vector<double> res ( x.size() );


    const double eps = 1.0e-6;
    //const double eps = _geom.get_h();

    boost::numeric::ublas::vector<double> xt;

    xt= x;

    for ( size_t i = 0; i< x.size(); i++ ) {
        xt ( i ) = x ( i )-eps;
        double d1 = f ( xt );

        xt ( i ) = x ( i ) +eps;
        double d2 = f ( xt );

        res ( i ) = ( d2-d1 ) / ( 2*eps );

        xt ( i ) = x ( i );
    }
    //std::cout<<"df = "<<res<<std::endl;
//     return res;
}



void ModRusselBeam::ddf_banded2 ( boost::numeric::ublas::matrix<double> &res, const boost::numeric::ublas::vector< double >& x ) {
//     boost::numeric::ublas::matrix<double> res ( x.size(), x.size() );
    res.clear();


    const double eps = 1e-6;

    for ( int i = 0; i< ( int ) x.size(); i++ ) {
        for ( int j = i; j< ( int ) x.size(); j++ ) {
            if ( ( ( i - j ) > 2 ) || ( ( j - i ) > 2 ) ) {
                res ( i, j ) = 0.0;
            } else if (
                ( ( ( i - j ) == 1 ) || ( ( j - i ) == 1 ) )
                &&
                ( i != ( int ) x.size() - 1 ) && ( j != ( int ) x.size() - 2 )
                &&
                ( i != ( int ) x.size() - 2 ) && ( j != ( int ) x.size() - 1 )
            ) {
                //if (  (i != x.size() - 1) && (i != x.size() - 2) )
                res ( i, j ) = 0.0;

            } else if ( i == j ) {
                boost::numeric::ublas::vector<double> xt;

                xt= x;

                xt ( i ) = x ( i ) + eps;
                const double ip = f ( xt );

                xt ( i ) = x ( i ) - eps;
                const double im = f ( xt );

                const double ic = f ( x );

                res ( i, j ) = ( ip - 2 * ic + im ) / ( eps * eps );
            } else {
                boost::numeric::ublas::vector<double> xt;

                xt= x;

                xt ( i ) = x ( i ) + eps;
                xt ( j ) = x ( j ) + eps;
                const double ipjp = f ( xt );

                xt ( i ) = x ( i ) + eps;
                xt ( j ) = x ( j ) - eps;
                const double ipjm = f ( xt );

                xt ( i ) = x ( i ) - eps;
                xt ( j ) = x ( j ) + eps;
                const double imjp = f ( xt );

                xt ( i ) = x ( i ) - eps;
                xt ( j ) = x ( j ) - eps;
                const double imjm = f ( xt );

                const double tmp = ( ipjp - ipjm - imjp + imjm ) / ( 4.0 * eps * eps );
                res ( i, j ) = tmp;
                res ( j, i ) = tmp;
            }
        }
    }

//     return res;
}




void ModRusselBeam::objective ( const boost::numeric::ublas::vector< double >& x, double& f, boost::numeric::ublas::vector< double >& df, boost::numeric::ublas::matrix< double >& ddf ) {
    //Implement the objective function here.
    //Input:	x: parameters
    //Output:	f: objective value
    //			df: gradient
    //			ddf: Hessian

    f = this->f ( x );
    this->df ( df, x );
    this->ddf_banded2 ( ddf, x );
}



/**
* The equality constraints. Require g(x)=0
*/
void ModRusselBeam::equalityConstraints ( const boost::numeric::ublas::vector< double >& x, size_t idx, double& g, boost::numeric::ublas::vector< double >& dg, boost::numeric::ublas::matrix< double >& ddg ) {

}




void ModRusselBeam::setHingeConstraintPointY ( const boost::numeric::ublas::vector< double >& x, std::size_t idx, boost::numeric::ublas::vector< double >& h, boost::numeric::ublas::matrix< double >& dh, boost::numeric::ublas::matrix< double >& ddh, int pIdx, int hBase ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = getObstacle()->get_yTCP ( planeTbeam );
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( x.size() );
    const double EPS = 10.0; // TODO problems if this is too small

    RW_ASSERT ( pIdx < ( int ) x.size() );
    
    // V component
    const double f0V = sin ( 0.0 );
    const double fLV = sin ( x[pIdx] );

    double sumV = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumV += sin ( x[i] );
    }
    double resV = 0.5 * hx * ( f0V + fLV ) + hx * sumV;
    
    
    // U component
    const double f0U = cos ( 0.0 );
    const double fLU = cos ( x[pIdx] );
    
    double sumU = 0.0;
    for ( int i = 0; i < ( int ) pIdx; i++ ) {
        sumU += cos ( x[i] );
    }
    double resU = ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU;

    
    
    const double uxTCPy =  get_uxTCPy(); // u2
    const double uyTCPy = get_uyTCPy(); // v2
    
//     std::cout << "uxTCPy: " << uxTCPy << std::endl;
//     std::cout << "uyTCPy: " << uyTCPy << std::endl;

//     const double &Ix = resU;
//     const double &Iy = resV;
    
    // h ( hBase ) = resU * uxTCPy        + resV * uyTCPy        + yTCP; // require h(x) > 0
    
    /* implementing:
     *   Iy <= Ytcp + Eps // don't go above
     *   Iy >= Ytcp - Eps // don't go below
     * 
     *  transformed to 
     * 
     *   -Iy >= -Ytcp - Eps
     *   Iy  >= Ytcp - Eps
     * 
     *  IP method require constraint formulated as h(x) > 0
     */
    h ( hBase ) = -( resU * uxTCPy        + resV * uyTCPy ) - yTCP + EPS; // NOTE sign of yTCP reversed!
    h ( hBase+1 ) = ( resU * uxTCPy        + resV * uyTCPy ) + yTCP + EPS; // NOTE sign of yTCP reversed! // OK
    
    
    
    
    
    
    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        if ( pIdx == i ) {
            dh ( hBase , i ) = (- 0.5 * hx * uyTCPy * cos ( x[i] ) + 0.5 * hx * uxTCPy * sin ( x[i] ) ) * 1.0 ;
            dh ( hBase+1, i ) = ( 0.5 * hx * uyTCPy * cos ( x[i] ) + 0.5 * hx * uxTCPy * sin ( x[i] ) ) * 1.0; 
        } else if ( i < pIdx ) {
            dh ( hBase, i ) = (- hx * uyTCPy * cos ( x[i] ) + hx * uxTCPy * sin ( x[i] )  ) * 1.0 ;  
            dh ( hBase+1, i ) = ( hx * uyTCPy * cos ( x[i] ) + hx * uxTCPy * sin ( x[i] ) ) * 1.0; 
        } else {
            dh ( hBase, i ) = 0.0;
            dh ( hBase+1, i ) = 0.0;
        }
    }
          
    if ( hBase  == ( int ) idx ) {
        ddh.clear();
        for ( int i = 0; i < ( int ) x.size(); i++ ) {
            for ( int j = 0; j < ( int ) x.size(); j++ ) {
                // only entries in the diagonal, i.e. d^2 f/ dx^2
                if ( i == j ) {
                    if ( pIdx == i ) {
                        ddh ( i, j ) =  ( 0.5 * hx * uxTCPy * cos ( x[i] ) + 0.5 * hx * uyTCPy * sin ( x[i] ) ) * 1.0; // OK
                    } else if ( i < pIdx ) {
                        ddh ( i, j ) =  ( 1.0 * hx * uxTCPy * cos ( x[i] ) + 1.0 * hx * uyTCPy * sin ( x[i] ) ) * 1.0; // OK
                    } else
                        ddh ( i, j ) = 0.0;
                }
            }
        }
    } 
    
    if ( hBase +1 == ( int ) idx ) {
        ddh.clear();
        for ( int i = 0; i < ( int ) x.size(); i++ ) {
            for ( int j = 0; j < ( int ) x.size(); j++ ) {
                // only entries in the diagonal, i.e. d^2 f/ dx^2
                if ( i == j ) {
                    if ( pIdx == i ) {
                        ddh ( i, j ) =  (-0.5 * hx * uxTCPy * cos ( x[i] ) - 0.5 * hx * uyTCPy * sin ( x[i] ) ) * 1.0; // OK
                    } else if ( i < pIdx ) {
                        ddh ( i, j ) =  (-1.0 * hx * uxTCPy * cos ( x[i] ) - 1.0 * hx * uyTCPy * sin ( x[i] ) ) * 1.0; // OK
                    } else
                        ddh ( i, j ) = 0.0;
                }
            }
        }
    } 
    
}






void ModRusselBeam::setInEqualityIntegralConstraintPoint ( const boost::numeric::ublas::vector< double >& x, std::size_t idx, boost::numeric::ublas::vector< double >& h, boost::numeric::ublas::matrix< double >& dh, boost::numeric::ublas::matrix< double >& ddh, int pIdx, int hBase ) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = getObstacle()->get_yTCP ( planeTbeam );
    const double hx = ( getGeometry()->get_b() - getGeometry()->get_a() ) / ( x.size() );   
    const double uxTCPy =  get_uxTCPy(); // u2
    const double uyTCPy = get_uyTCPy(); // v2

    RW_ASSERT ( pIdx < ( int ) x.size() );
    
    
    
    
    
    
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

    
    


    
    

    /* tip constraint h: 
     *  endCon = resU uxTCPy + resV uyTCPy >= -yTCP;
     */
    h ( hBase ) = resU * uxTCPy        + resV * uyTCPy        + yTCP; // require h(x) > 0


    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        if ( pIdx == i ) {
            dh ( hBase, i ) = 0.5 * hx * uyTCPy * cos ( x[i] ) - 0.5 * hx * uxTCPy * sin ( x[i] );
        } else if ( i < pIdx ) {
            dh ( hBase, i ) = hx * uyTCPy * cos ( x[i] ) - hx * uxTCPy * sin ( x[i] );
        } else
            dh ( hBase, i ) = 0.0;
    }



    // tip constraint, ddh
    if ( hBase == ( int ) idx ) {
        ddh.clear();
        for ( int i = 0; i < ( int ) x.size(); i++ ) {
            for ( int j = 0; j < ( int ) x.size(); j++ ) {
                // only entries in the diagonal, i.e. d^2 f/ dx^2
                if ( i == j ) {
                    if ( pIdx == i ) {
                        // ddh(i, j) = -0.5 * hx * sin( x[i] );
                        ddh ( i, j ) = -0.5 * hx * uxTCPy * cos ( x[i] ) - 0.5 * hx * uyTCPy * sin ( x[i] );
                    } else if ( i < pIdx ) {
                        //ddh(i, j) = -1.0 * hx * sin( x[i] );
                        ddh ( i, j ) = -1.0 * hx * uxTCPy * cos ( x[i] ) - 1.0 * hx * uyTCPy * sin ( x[i] );
                    } else
                        ddh ( i, j ) = 0.0;
                }
            }
        }
    } 
};



void ModRusselBeam::setInEqualityNoUpwardsEtaConstraint (
    const boost::numeric::ublas::vector< double >& x,
    size_t idx,
    boost::numeric::ublas::vector< double >& h,
    boost::numeric::ublas::matrix< double >& dh,
    boost::numeric::ublas::matrix< double >& ddh,
    int hBase
) {
    const double uxTCPy =  get_uxTCPy();
    const double uyTCPy = get_uyTCPy();
    // run through all points and formulate the 'no-pointing-upwards' constraint
    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        h ( hBase + i ) = - uyTCPy * sin ( x[i] ) - uxTCPy * cos ( x[i] );

        // dh
        for ( int j = 0; j < ( int ) x.size(); j++ ) {
            if ( j == i ) {
                // - _uyTCPy * sin( x[i] ) - _uxTCPy * cos( x[i] )
                //dh( i+1, j) = -cos ( x[i] );
                dh ( hBase + i, j ) = -uyTCPy * cos ( x[i] ) + uxTCPy * sin ( x[i] );
            } else
                dh ( hBase + i, j ) = 0.0;
        }
    }
    
    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        // ddh
        if ( hBase + i == ( int ) idx ) {
            // the 'no-pointing-upwards' constraint for point at xi = i*h, has idx=i+1

            // if idx=1 then we are dealing with the point at i=0

            // the constraint is there
            // -sin( x[0] )

            // only non-zero entry in the hessian is at i=j=0
            ddh.clear();
            //ddh(i, i) =  sin ( x[i] );
            // _uxTCPy Cos[x] + _uyTCPy Sin[x]
            ddh ( i, i ) =  uxTCPy * cos ( x[i] ) + uyTCPy * sin ( x[i] );
        }
    }
}



/**
* The inequality constraints. Require h(x) > 0
//Implement the objective function here.
//Input:	x: parameters
//			idx: index of the active constraint
//Output:		h: constraint value
//			dh: constraint gradient
//			ddh: constraint Hessian
*/
void ModRusselBeam::inEqualityConstraints (
    const boost::numeric::ublas::vector< double >& x,
    size_t idx,
    boost::numeric::ublas::vector< double >& h,
    boost::numeric::ublas::matrix< double >& dh,
    boost::numeric::ublas::matrix< double >& ddh
) {
    std::vector<int> integralConstraintIdxList = getIntegralIndices();
    
    int hBase = 0;
    for ( int i = 0; i < ( int ) integralConstraintIdxList.size(); i++ ) {
        int pIdx = integralConstraintIdxList[i];
        setInEqualityIntegralConstraintPoint ( x, idx, h, dh, ddh, pIdx, hBase++ );
    }

    if ( getUseHingeConstraint() ) {      
        int pIdx = getN() - 1; // hinge at x=L
//         std::cout << "x: " << x << std::endl;
//         std::cout << "idx: " << idx << std::endl;
//         std::cout << "pIdx: " << pIdx << std::endl;
//         std::cout << "hBase: " << hBase << std::endl;
        setHingeConstraintPointY( x, idx, h, dh, ddh, pIdx, hBase );
        hBase += 2;
    }
    

    if ( getUseNoUpwardConstraint() ) {
        setInEqualityNoUpwardsEtaConstraint ( x, idx, h, dh, ddh, hBase++ ); // h[1] : h[x.size() + 1]
    }
    
    


//     std::cout << "h: " << h << std::endl;
//     std::cout << "dh: " << dh << std::endl;
//     std::cout << "ddh: " << ddh << std::endl;
    
//     RW_ASSERT(false);
}



void ModRusselBeam::solve ( boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector< double >& U, boost::numeric::ublas::vector< double >& V ) {
    cout << "ModRusselBeam::solve()" << endl;

    if ( get_nIntegralConstraints() < 0 )
        RW_THROW ( "Number of integral constraints cannot be negative" );

    if ( get_nIntegralConstraints() > getN() )
        RW_THROW ( "Number of integral constraints must be less or equal to M-1" );

    
    computeIntegralIndicies();
    

    const size_t N = getN(); //Dimensions of parameter space

    size_t L =    getUseNoUpwardConstraint() == true      ?    get_nIntegralConstraints() + N : get_nIntegralConstraints();
    
    if (getUseHingeConstraint())
        L += 2;
    

    boost::numeric::ublas::vector<double> xinit ( N );
    for ( int i = 0; i < ( int ) N; i++ ) {
        xinit[i] = xinituser[i+1];
    }

    std::cout << "xinituser: " << xinituser << std::endl;
    std::cout << "DIVIDER: " << DIVIDER << std::endl;

    NFCALLS = 0;
    InteriorPointOptimizer iop ( N, L,
                                 boost::bind ( &ModRusselBeam::objective, this, _1, _2, _3, _4 ),
                                 boost::bind ( &ModRusselBeam::inEqualityConstraints, this, _1, _2, _3, _4, _5 )
                               );
    iop.setAccuracy ( getAccuracy() );
    iop.setMuDecrementFactor(getMuDecrementFactor());
    iop.setMuStart(getMuStart());
    boost::numeric::ublas::vector<double> res = iop.solve ( xinit );
    std::cout << "NFCALLS: " << NFCALLS << std::endl;

    const double Ee = f_elastic ( res );
    std::cout << "Ee: " << Ee << " [kg mm^2/s^2] = " << Ee * 1.0e-6 << " [J]" << std::endl;

    integrateAngleU ( U, res );
    integrateAngleV ( V, res );

    xinituser[0] = 0.0;
    for ( int i = 0; i < ( int ) res.size(); i++ )
        xinituser[i+1] = res[i];
}
