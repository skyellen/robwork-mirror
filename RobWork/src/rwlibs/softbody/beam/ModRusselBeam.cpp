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

#include "ModRusselBeam.hpp"

#include "rw/math/Math.hpp"


#include "rwlibs/softbody/numerics/TrapMethod.hpp"
#include "rwlibs/softbody/numerics/FdUtil.hpp"

using namespace std;
using namespace rw::math;
using namespace boost::numeric::ublas;
using namespace rwlibs::softbody;


#include "rwlibs/softbody/numerics/InteriorPointOptimizer.hpp"
#include "BeamGeometry.hpp"
#include "rwlibs/softbody/numerics/Interpolation.hpp"
#include "rwlibs/softbody/util/psplot.h"

static int NFCALLS = 0;


const char DIVIDER[] = "--------------------------------------------------------------------------------";

ModRusselBeam::ModRusselBeam (
    boost::shared_ptr< rwlibs::softbody::BeamGeometry > geomPtr,
    boost::shared_ptr< rwlibs::softbody::BeamObstaclePlane > obstaclePtr,
    int M,
    double accuracy,
    bool useNoUpwardConstraint
) : _geomPtr ( geomPtr ), _obstaclePtr ( obstaclePtr ), _M ( M ), _accuracy ( accuracy ), _a ( M ), _da ( M ), _useNoUpwardConstraint ( useNoUpwardConstraint ) {
    assert ( _M >= 2 );
}

Transform3D< double > ModRusselBeam::get_planeTbeam ( void ) const {
    const rw::math::Transform3D<> &Tbeam = _geomPtr->getTransform();
    rw::math::Transform3D<> planeTbeam = _obstaclePtr->compute_planeTbeam ( Tbeam );

    return planeTbeam;
}


double ModRusselBeam::get_thetaTCP ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double thetaTCP = _obstaclePtr->get_thetaTCP ( planeTbeam );

    return thetaTCP;
}



double ModRusselBeam::get_yTCP ( void ) const {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = _obstaclePtr->get_yTCP ( planeTbeam );

    return yTCP;
}





double ModRusselBeam::get_h ( void )  const {
    // TODO calculate once and set var
    double a = _geomPtr->get_a();
    double b = _geomPtr->get_b();

    return ( b-a ) / ( getM()-1 );
}


int ModRusselBeam::getM ( void )  const {
    return _M;
}


int ModRusselBeam::getN ( void )  const {
    return getM() -1;
}


struct RusselIntegrand {
    RusselIntegrand (
        const BeamGeometry &geom,
        const boost::numeric::ublas::vector<double>& a,
        const boost::numeric::ublas::vector<double>& da
    ) :
        _geom ( geom ),
        _a ( a ),
        _da ( da ) {

    };



    double operator() ( const int i ) const {
        assert ( i < ( int ) _a.size() );
        assert ( i < ( int ) _da.size() );

        const double &ax = _a[i];
        const double &dax = _da[i];

        // BUG: something is wrong with the equations regarding gravity as we need to hack the signs
        const double &g1 = _geom.g1()  * ( -1.0 );
        const double &g2 = _geom.g2() * ( -1.0 );

        const double b0 = _geom.b0 ( i );
        const double b1 = _geom.b1 ( i );

        const double B0 = _geom.B0 ( i );

        const double c2 = _geom.c2 ( i );
        const double c3 = _geom.c3 ( i );
        const double c4 = _geom.c4 ( i );

        const double x = i * _geom.get_h();


        const double val = ( g1 * B0 + g2*b1 ) * cos ( ax ) + ( g2*B0 - g1*b1 ) * sin ( ax ) +
                           4 * c2 * pow ( dax, 2.0 ) + 4 * c3 * pow ( dax, 3.0 ) + c4 * pow ( dax, 4.0 ) - g1*b0 * x - g2*b1;

        return val;
    }

private:
    const BeamGeometry &_geom;
    const boost::numeric::ublas::vector<double>& _a;
    const boost::numeric::ublas::vector<double>& _da;
};



//This is the object function
double ModRusselBeam::f ( const boost::numeric::ublas::vector<double>& x ) {
    //const int N = getN();
    const int M = getM();
    const double h = get_h();

    assert ( ( int ) x.size() == getN() );

    assert ( ( int ) _a.size() == getM() );



    _a[0] = 0.0;
    for ( int i = 1; i < M; i++ )
        _a[i] = x[i-1];


    FdUtil::vectorDerivative ( _a, _da, get_h() ) ;


    RusselIntegrand intgr ( *_geomPtr, _a, _da );
    double val = TrapMethod::trapezMethod<RusselIntegrand> ( intgr, M, h );

    NFCALLS++;

    return val;
}

//Computing gradient for object function
boost::numeric::ublas::vector<double> ModRusselBeam::df ( const boost::numeric::ublas::vector<double>& x ) {
    boost::numeric::ublas::vector<double> res ( x.size() );


    const double eps = 1e-6;
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
    return res;
}
/*
//Computing hessian for object function
boost::numeric::ublas::matrix<double> ModRusselBeam::ddf(const boost::numeric::ublas::vector<double>& x) {
	 boost::numeric::ublas::matrix<double> res(x.size(), x.size());

	const double eps = 1e-6;
	//const double eps = _geom.get_h();
// 	std::cout << "_geom.get_h(): " << _geom.get_h() << std::endl;

	boost::numeric::ublas::vector<double> xt = x;


	for (size_t i = 0; i<x.size(); i++) {
		xt(i) = x(i)-eps;
		boost::numeric::ublas::vector<double> g1 = df(xt);
		xt(i) = x(i)+eps;
		boost::numeric::ublas::vector<double> g2 = df(xt);


		const boost::numeric::ublas::vector<double> tmp = (g2-g1)/(2*eps);

		for (size_t j = 0; j<x.size(); j++) {
			res(i,j) = tmp(j);
		}
		xt(i) = x(i);
	}

	std::cout<<"DDF org = "<<res<<std::endl;



	return res;
}
*/

double ModRusselBeam::diff_i ( const boost::numeric::ublas::vector< double >& x, const int i ) {
    const double eps = 1e-6;

    boost::numeric::ublas::vector<double> xt = x;

    xt ( i ) = x ( i )-eps;
    double d1 = f ( xt );

    xt ( i ) = x ( i ) +eps;
    double d2 = f ( xt );

    return ( d2-d1 ) / ( 2*eps );
}



boost::numeric::ublas::matrix< double > ModRusselBeam::ddf_banded2 ( const boost::numeric::ublas::vector< double >& x ) {
    boost::numeric::ublas::matrix<double> res ( x.size(), x.size() );
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

    //std::cout << "res: " << res << std::endl;

    /*
    std::cout<<"DDF banded = "<<res<<std::endl;

    std::cout << "res.size(): " << res.size1() << std::endl;
    for (int i = 0; i < res.size1(); i++) {
    for (int j = 0; j < res.size2(); j++) {
        if ( abs( res(i,j) ) > 0.001 ) {
    	std::cout << 1 << " ";
        }
        else {
    	std::cout << 0  << " ";
        }
    }
    std::cout << std::endl;
    }

    assert(false);
    */
    return res;
}




void ModRusselBeam::objective ( const boost::numeric::ublas::vector< double >& x, double& f, boost::numeric::ublas::vector< double >& df, boost::numeric::ublas::matrix< double >& ddf ) {
    //Implement the objective function here.
    //Input:	x: parameters
    //Output:	f: objective value
    //			df: gradient
    //			ddf: Hessian

    f = this->f ( x );
    df = this->df ( x );
    ddf = this->ddf_banded2 ( x );
    //ddf = this->ddf(x);

// 	std::cout << "f: " << f << std::endl;
// 	std::cout << "df: " << df << std::endl;
// 	std::cout << "ddf: " << ddf << std::endl;
}

/**
* The equality constraints. Require g(x)=0
*/
void ModRusselBeam::equalityConstraints ( const boost::numeric::ublas::vector< double >& x, size_t idx, double& g, boost::numeric::ublas::vector< double >& dg, boost::numeric::ublas::matrix< double >& ddg ) {

}


void ModRusselBeam:: setInEqualityIntegralConstraint (
    const boost::numeric::ublas::vector< double >& x,
    size_t idx,
    boost::numeric::ublas::vector< double >& h,
    boost::numeric::ublas::matrix< double >& dh,
    boost::numeric::ublas::matrix< double >& ddh
) {
    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = _obstaclePtr->get_yTCP ( planeTbeam );
    double thetaTCP = _obstaclePtr->get_thetaTCP ( planeTbeam );

    const double hx = ( _geomPtr->get_b() - _geomPtr->get_a() ) / ( x.size() );
    //const int L = h.size();

    // V component
    const double f0V = sin ( 0.0 );
    const double fLV = sin ( x[x.size() - 1] );

    double sumV = 0.0;
    for ( int i = 0; i < ( int ) x.size() -1; i++ ) {
        sumV += sin ( x[i] );
    }
    double resV = ( hx / 2.0 ) * ( f0V + fLV ) + hx * sumV;

    // U component
    const double f0U = cos ( 0.0 );
    const double fLU = cos ( x[x.size() - 1] );

    double sumU = 0.0;
    for ( int i = 0; i < ( int ) x.size() -1; i++ ) {
        sumU += cos ( x[i] );
    }
    double resU = ( hx / 2.0 ) * ( f0U + fLU ) + hx * sumU;

    const double uxTCPy =  _geomPtr->get_uxTCPy();
    const double uyTCPy = _geomPtr->get_uyTCPy();

    // endCon = UconEnd uxTCPy + VconEnd uyTCPy >= -yTCP;
    h ( 0 ) = resU * uxTCPy        + resV * uyTCPy        + yTCP; // require h(x) > 0

    // tip constraint dh
    dh ( 0, 0 ) = 0.5 * hx * uyTCPy * cos ( x[0] ) - 0.5 * hx * uxTCPy * sin ( x[0] );
    for ( int i = 1; i < ( int )  x.size() - 1; i++ ) {
        dh ( 0, i ) = hx * uyTCPy * cos ( x[i] ) - hx * uxTCPy * sin ( x[i] );
    }
    dh ( 0, 0 ) = 0.5 * hx * uyTCPy * cos ( x[x.size() -1] ) - 0.5 * hx * uxTCPy * sin ( x[x.size() -1] );


    // tip constraint, ddh
    if ( 0 == idx ) {
        ddh.clear();
        for ( int i = 0; i < ( int ) x.size(); i++ ) {
            for ( int j = 0; j < ( int ) x.size(); j++ ) {
                // only entries in the diagonal, i.e. d^2 f/ dx^2
                if ( i == j ) {
                    if ( 0 == i || ( x.size() -1 ) == i ) {
                        // ddh(i, j) = -0.5 * hx * sin( x[i] );
                        ddh ( i, j ) = -0.5 * hx * uxTCPy * cos ( x[i] ) - 0.5 * hx * uyTCPy * sin ( x[i] );
                    } else {
                        //ddh(i, j) = -1.0 * hx * sin( x[i] );
                        ddh ( i, j ) = -1.0 * hx * uxTCPy * cos ( x[i] ) - 1.0 * hx * uyTCPy * sin ( x[i] );
                    }
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
    boost::numeric::ublas::matrix< double >& ddh
) {
    const double uxTCPy =  _geomPtr->get_uxTCPy();
    const double uyTCPy = _geomPtr->get_uyTCPy();
    // run through all points and formulate the 'no-pointing-upwards' constraint
    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        h ( i+1 ) = - uyTCPy * sin ( x[i] ) - uxTCPy * cos ( x[i] );

        // dh
        for ( int j = 0; j < ( int ) x.size(); j++ ) {
            if ( j == i ) {
                // - _uyTCPy * sin( x[i] ) - _uxTCPy * cos( x[i] )
                //dh( i+1, j) = -cos ( x[i] );
                dh ( i+1, j ) = -uyTCPy * cos ( x[i] ) + uxTCPy * sin ( x[i] );
            } else
                dh ( i+1, j ) = 0.0;
        }
    }

    for ( int i = 0; i < ( int ) x.size(); i++ ) {
        // ddh
        if ( i + 1 == ( int ) idx ) {
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

    setInEqualityIntegralConstraint ( x, idx, h, dh, ddh ); // h[0]

    if ( _useNoUpwardConstraint )
        setInEqualityNoUpwardsEtaConstraint ( x, idx, h, dh, ddh ); // h[1] : h[x.size() + 1]

// 	std::cout << "idx: " << idx << std::endl;
// 	std::cout << "x: " << x << std::endl;
//   	std::cout << "h: " << h << std::endl;
//   	std::cout << "dh: " << dh << std::endl;
//   	std::cout << "ddh: " << ddh << std::endl;
// 	std::cout << "DIVIDER: " << DIVIDER << std::endl;
}



/*TODO: how can we detect if we need the constraints?
    For LD we need the no-upwards-constraint to keep it stable when in flat contact with the table


    user or automatic choice of constraints?








*/
//TODO: strong impact of discretization step on the U and V integrated constraint...Extrapolate?
//TODO: better adaptive optimazation, can we trust our error estimate? seems accurate tho...
void ModRusselBeam::solve ( boost::numeric::ublas::vector< double >& xinituser, boost::numeric::ublas::vector< double >& U, boost::numeric::ublas::vector< double >& V ) {
    cout << "ModRusselBeam::solve()" << endl;

    const rw::math::Transform3D<> planeTbeam = get_planeTbeam();
    double yTCP = _obstaclePtr->get_yTCP ( planeTbeam );
    double thetaTCP = _obstaclePtr->get_thetaTCP ( planeTbeam );
    
    Vector3D<> G = _geomPtr->getG();
    
    double g1 = _geomPtr->g1();
    double g2 = _geomPtr->g2();
    
    std::cout << "G: " << G << std::endl;
    
    std::cout << "g1: " << g1 << std::endl;
    std::cout << "g2: " << g2 << std::endl;

    std::cout << "yTCP: " << yTCP << std::endl;
    std::cout << "thetaTCP: " << thetaTCP << std::endl;


    const size_t N = getN(); //Dimensions of parameter space

    //const size_t M = 0; // Number of equality constraints

    const size_t L = 	_useNoUpwardConstraint == true 		?	 1 + N : 1;

    // all constraints
    //const size_t L = 1 + N; //Number of inequality constraints

    // disable no-upwards constraint
    //const size_t L = 1 ; //Number of inequality constraints


//  	std::cout << "N: " << N << std::endl;

    boost::numeric::ublas::vector<double> xinit ( N );
    for ( int i = 0; i < ( int ) N; i++ ) {
        xinit[i] = xinituser[i+1];
    }

    std::cout << "xinituser: " << xinituser << std::endl;
// 	std::cout << "xinit: " << xinit << std::endl;

    std::cout << "DIVIDER: " << DIVIDER << std::endl;

    InteriorPointOptimizer iop ( N, L,
                                 boost::bind ( &ModRusselBeam::objective, this, _1, _2, _3, _4 ),
                                 boost::bind ( &ModRusselBeam::inEqualityConstraints, this, _1, _2, _3, _4, _5 )

                               );
    iop.setAccuracy ( _accuracy );
    boost::numeric::ublas::vector<double> res = iop.solve ( xinit );

// 	std::cout << "DIVIDER: " << DIVIDER << std::endl;

// 	std::cout << "res: " << res << std::endl;

    const double h = ( _geomPtr->get_b() - _geomPtr->get_a() ) / res.size();

    U[0] = 0.0;
    for ( int end = 0; end < ( int )  res.size(); end++ ) {
        const double f0 = cos ( 0.0 );
        const double fL = cos ( res[end] );

        double sum = 0.0;
        for ( int i = 0; i < ( int ) end; i++ )
            sum += cos ( res[i] );

        U[end+1] = ( h / 2.0 ) * ( f0 + fL ) + h * sum;
    }

    V[0] = 0.0;
    for ( int end = 0; end < ( int ) res.size(); end++ ) {
        const double f0 = sin ( 0.0 );
        const double fL = sin ( res[end] );

        double sum = 0.0;
        for ( int i = 0; i < ( int ) end; i++ )
            sum += sin ( res[i] );

        V[end+1] = ( h / 2.0 ) * ( f0 + fL ) + h * sum;
    }

    // rotate curve according to thetaTCP
    /*
    boost::numeric::ublas::vector<double> Urot ( U.size() );
    boost::numeric::ublas::vector<double> Vrot ( V.size() );

    const Rotation3D<> &rot = _geomPtr->getTransform().R();

    for ( int i = 0; i < ( int ) U.size(); i++ ) {
        Urot[i] = rot ( 0,0 ) * U[i] + rot ( 0, 1 ) * V[i];
        Vrot[i] = rot ( 1,0 ) * U[i] + rot ( 1, 1 ) * V[i];
    }
    // return the rotated version to the user
    U = Urot;
    V = Vrot;
    
    xinituser[0] = thetaTCP;
    for ( int i = 0; i < ( int ) res.size(); i++ )
        xinituser[i+1] = res[i] + thetaTCP;
    */

    
    
    
    xinituser[0] = 0.0;
    for ( int i = 0; i < ( int ) res.size(); i++ )
        xinituser[i+1] = res[i];


    std::cout << "NFCALLS: " << NFCALLS << std::endl;
}




/*
struct AngleIntegrandU {
    AngleIntegrandU(
    const boost::numeric::ublas::vector<double> &a,
    const Geometry &geom
    ) :
    _a(a),
    _geom(geom)
    {};

    double operator() (const int i) const {
	return cos(_a[i]);
	//return _a[i];
	}

	private:
	    const boost::numeric::ublas::vector<double> &_a;
	    const Geometry &_geom;
	    };
	    */

/*
struct AngleIntegrandV {
    AngleIntegrandV(
    const boost::numeric::ublas::vector<double> &a,
    const Geometry &geom
    ) :
    _a(a),
    _geom(geom)
    {};

    double operator() (const int i) const {
	return sin(_a[i]);
    }

    private:
	const boost::numeric::ublas::vector<double> &_a;
	const Geometry &_geom;
	};
	*/


/*
boost::numeric::ublas::vector<double> BendOptimizer::integrateAngleU(const boost::numeric::ublas::vector<double> a)  {
    const int M = a.size();
    const double h =  (_geom.get_b() - _geom.get_a() ) / (M -1.0);

    std::cout << "M: " << M << std::endl;
    std::cout << "h: " << h << std::endl;

    std::cout << "a: " << a << std::endl;

    const double b = 110;
    const double aa = 0.0;
    std::cout << "(b-aa)/M: " << (b-aa)/M << std::endl;

    AngleIntegrandU intgr( a, _geom);


    boost::numeric::ublas::vector<double> res(M +1);

    //assert(res.size() == a.size()); // TODO FIXME, using wrong vector length

    res[0] = 0.0;
    for (int k = 1; k < M +1; k++) {
	res[k] = TrapMethod::trapezMethod<AngleIntegrandU>(intgr, k, h);
	}


	return res;
	};
	*/
/*
boost::numeric::ublas::vector<double> BendOptimizer::integrateAngleV(const boost::numeric::ublas::vector<double> a)  {
    const int M = a.size();
    const double h = (_geom.get_b() - _geom.get_a() ) / (M -1.0);

    AngleIntegrandV intgr( a, _geom);


    boost::numeric::ublas::vector<double> res(M+1);

    //assert(res.size() == a.size()); // TODO FIXME, using wrong vector length

    res[0] = 0.0;
    for (int k = 1; k < M +1; k++) {
	res[k] = TrapMethod::trapezMethod<AngleIntegrandV>(intgr, k, h);
	}


	return res;
	};
	*/


/*
size_t N = x.size();

for(int i=0;i<N;i++) {
    dh(idx, i)=0.0;
    for(int j=0;j<N;j++) {
	ddh(i, j)=0.0;
	}
	}

	h(0) = 10 - x[0];
	h(1) =  x[0];

	// only 1 variable here
	if (0 == idx) { // 1st constraint
	    dh(idx, 0) = -1;
	    ddh(idx, 0) = 0;
	    }

	    if (1 == idx) { // 2nd constraint
		dh(idx, 0) = 1;
		ddh(0, 0) = 0;
		}
		*/

