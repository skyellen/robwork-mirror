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


#include "InterpolatorTestSuite.hpp"

#include <rw/interpolator/StraightSegment.hpp>
#include <rw/interpolator/StraightInterpolator.hpp>

#include <rw/interpolator/CubicSegment.hpp>
#include <rw/interpolator/CubicSplineInterpolator.hpp>

#include <rw/interpolator/Trajectory.hpp>
#include <rw/interpolator/TrajectoryIterator.hpp>

#include <rw/math/EAA.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Rotation3D.hpp>

#include <boost/numeric/bindings/lapack/spsv.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>
#include <boost/numeric/bindings/traits/ublas_banded.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>

#include <string>

using namespace boost::unit_test;

using namespace rw;
using namespace rw::math;
using namespace rw::interpolator;

namespace lapack = boost::numeric::bindings::lapack;
namespace ublas = boost::numeric::ublas;

bool close_enough(Q v1,Q v2){
    bool enough = true;
    for(unsigned int i = 0; i<v1.size(); i++)
        enough = enough && fabs(v1[i]-v2[i])<1e-5;
    return enough;
}

/*extern "C" {
    void spttrs_(int *n, int *nrhs, float *d, float *e, float *b, int *ldb, long *info);
}

extern "C" {
    void sgtsv_(int *n, int *nrhs, float *dl, float *d, float *du, float *b, int *ldb, long *info);
}

extern "C" {
    void sptsv_(int *n, int *nrhs, float *d, float *e, float *b, int *ldb, long *info);
}

extern "C" {
    void spbtrs_(char *uplo, int *n, int *kd, int *nrhs, float *ab, int *ldab, float *b, int *ldb, long *info);
}*/

void InterpolatorTest(){
    std::cout << "******************* Testing Interpolator functions!!" << std::endl;
    //Test StraightSegment
    {
        Q A(2),B(2);
        A[0] = 1; A[1] = 1;
        B[0] = 10; B[1] = 5;
        double s = 10;
        StraightSegment func(A, B-A, s);

        BOOST_CHECK(func.getLength() == 10);
        BOOST_CHECK(func.getX(0)(0) == A[0]);
        BOOST_CHECK(func.getX(0)(1) == A[1]);
        BOOST_CHECK(func.getX(func.getLength())(0) == B[0]);
        BOOST_CHECK(func.getX(func.getLength())(1) == B[1]);
        BOOST_CHECK(func.getX(func.getLength()/2)(0) == A[0]+(B[0]-A[0])/2);
        BOOST_CHECK(func.getX(func.getLength()/2)(1) == A[1]+(B[1]-A[1])/2);

    }
    //Test StraightInterpolator
    {
        Q A(2),B(2),C(2),D(2);
        A[0] = 1; A[1] = 1;
        B[0] = 10; B[1] = 5;
        double tb = 2;
        C[0] = 3; C[1] = 7;
        double tc = 3;
        D[0] = 7; D[1] = 4;
        double td = 1;

        StraightInterpolator interp(A,B,tb);

        BOOST_CHECK(interp.getX(0)(0) == A[0]);
        BOOST_CHECK(interp.getX(0)(1) == A[1]);

        BOOST_CHECK(interp.getX(tb/2)(0) == (B[0]+A[0])/2.0);
        BOOST_CHECK(interp.getX(tb/2)(1) == (B[1]+A[1])/2.0);

        BOOST_CHECK(interp.getX(tb)(0) == B[0]);
        BOOST_CHECK(interp.getX(tb)(1) == B[1]);


        interp.addVia(C, tc);
        BOOST_CHECK(interp.getX((tc+tb)/2)(0) == (C[0]+B[0])/2.0);
        BOOST_CHECK(interp.getX((tc+tb)/2)(1) == (C[1]+B[1])/2.0);

        BOOST_CHECK(interp.getX(tc)(0) == C[0]);
        BOOST_CHECK(interp.getX(tc)(1) == C[1]);

        BOOST_CHECK(interp.insertVia(D, td));
        BOOST_CHECK(interp.getX(td/2)(0) == (D[0]+A[0])/2.0);
        BOOST_CHECK(interp.getX(td/2)(1) == (D[1]+A[1])/2.0);

        BOOST_CHECK(interp.getX(td)(0) == D[0]);
        BOOST_CHECK(interp.getX(td)(1) == D[1]);

        BOOST_CHECK(interp.getX((tb+td)/2)(0) == (B[0]+D[0])/2.0);
        BOOST_CHECK(interp.getX((tb+td)/2)(1) == (B[1]+D[1])/2.0);

        interp.removeVia(2); //Remove C

        BOOST_CHECK(interp.getX((td+tc)/2)(0) == (D[0]+C[0])/2.0);
        BOOST_CHECK(interp.getX((td+tc)/2)(1) == (D[1]+C[1])/2.0);

        BOOST_CHECK(interp.getX(tc)(0) = C[0]);
        BOOST_CHECK(interp.getX(tc)(1) = C[1]);

        InterpolatorIterator it = interp.getIterator();
        BOOST_CHECK((*it)(0) == A[0]);
        BOOST_CHECK((*it)(1) == A[1]);
        it += td;
        BOOST_CHECK((*it)(0) == D[0]);
        BOOST_CHECK((*it)(1) == D[1]);
        it += (tc-td)/2;
        BOOST_CHECK((*it)(0) == (D[0]+C[0])/2);
        BOOST_CHECK((*it)(1) == (D[1]+C[1])/2);

        std::vector<std::pair<Q, double> > dataset;
        dataset.push_back(std::pair<Q, double>(D, td));
        dataset.push_back(std::pair<Q, double>(C, tc));

        StraightInterpolator interp2(A, dataset);
        BOOST_CHECK(close_enough(interp.getX(0), interp2.getX(0)));
        BOOST_CHECK(close_enough(interp.getX(0.5), interp2.getX(0.5)));
        BOOST_CHECK(close_enough(interp.getX(1.5), interp2.getX(1.5)));


    }

    //Test TrajectoryIterator
    {
        Q A(2), B(2), C(2), D(2);
        A[0] = 0; A[1] = 0;
        B[0] = 1; B[1] = 1;
        C[0] = 3; C[1] = 2;
        C[0] = 5; C[1] = 6;
        double Tb = 1;
        double Tc = 2;
        double Td = 2;

        StraightInterpolator si1(A,B, Tb);
        StraightInterpolator si2(B,C, Tc);
        StraightInterpolator si3(C,D, Td);
        Trajectory P;
        P.add(&si1);
        P.add(&si2);
        P.add(&si3);
        TrajectoryIterator p_iter = P.getIterator();
        // Check Start and end points
        BOOST_CHECK(close_enough( (*p_iter), A));
        p_iter += si1.getLength();
        BOOST_CHECK(close_enough( (*p_iter), B));
        p_iter += si2.getLength();
        BOOST_CHECK(close_enough( (*p_iter), C));
        p_iter += si3.getLength()/2;
        BOOST_CHECK(close_enough( (*p_iter), (C+D)/2));
        p_iter += si3.getLength()/2;
        BOOST_CHECK(close_enough( (*p_iter), D));


    }

    //Test CubicSplineInterpolator
    {
        // Create the viaPoints
        Q p0(2),p1(2),p2(2),p3(2);
        p0[0] = 0; p0[1] = 0;
        p1[0] = 1; p1[1] = 2;
        p2[0] = -1; p2[1] = 3;
        p3[0] = 0; p3[1] = 1;

        std::vector< std::pair<Q, double> > viaPoints;
        viaPoints.push_back(std::pair<Q, double>(p1,1));
        viaPoints.push_back(std::pair<Q, double>(p2,2));
        viaPoints.push_back(std::pair<Q, double>(p3,4));

        // Give the viaPoints as argument to the CubicSplineInterpolator
        CubicSplineInterpolator cubicInterpolator(p0, viaPoints);

        // Get the InterpolatorIterator and run through the path
        InterpolatorIterator iter = cubicInterpolator.getIterator();
        BOOST_CHECK(close_enough(*iter, p0));
        iter += 1;
        BOOST_CHECK(close_enough(*iter, p1));
        iter += 1;
        BOOST_CHECK(close_enough(*iter, p2));
        iter += 2;
        BOOST_CHECK(close_enough(*iter, p3));
    }
    //Code below does not really test anything
  /*  {
        Q p0(2),p1(2),p2(2),p3(2);
        p0[0] = 0; p0[1] = 0;
        p1[0] = 1; p1[1] = 2;
        p2[0] = -1; p2[1] = 3;
        p3[0] = 0; p3[1] = 1;
        std::vector< std::pair<Q, double> > viaPoints;
        viaPoints.push_back(std::pair<Q, double>(p1, 1));
        viaPoints.push_back(std::pair<Q, double>(p2,2));
        viaPoints.push_back(std::pair<Q, double>(p3,3));
        CubicSplineInterpolator interp(p0, viaPoints);

        int N = 4;
        int NRHS = 1;
        int LDB = N;

        int KD = 1; // The number of superdiagonals
        int LDAB = KD + 1; // The leading dimension of array AB
        //viapoints {{0, 0}, {1, 2}, {-1, 3}, {0, 1}}
        // Create the equations
        float X1[4],X2[4];
        X1[0] = 0;X1[1] = 1;X1[2] = -1;X1[3] = 0;
        X2[0] = 0;X2[1] = 2;X2[2] =  3;X2[3] = 1;

        float B[4],D[4],DL[3],DU[3],E[3]; // AB[LDAB][N];
        D[0] = 2;D[1] = 4;D[2] = 4;D[3] = 2;
        E[0] = 1;E[1] = 1;E[2] = 1;
        //AB(kd+1+i-j,j) = U(i,j)

        long info;
        int i;

        for(i=0;i<3;i++){
            DL[i] = E[i];
            DU[i] = E[i];
        }

        B[0] = 3*(X1[1]-X1[0]);
        B[1] = 3*(X1[2]-X1[0]);
        B[2] = 3*(X1[3]-X1[1]);
        B[3] = 3*(X1[3]-X1[2]);
        float a[4],b[4],c[4],d[4];

        std::cout << "Solve: ";
        for(i=0;i<N;i++)
            std::cout << B[i] << " ";
        std::cout << std::endl;

        //spttrs_(&N,&NRHS,D,E,&B[0][0],&LDB,&info);
//        sgtsv_(&N,&NRHS,DL,D,DU,&B[0][0],&LDB,&info);
        sptsv_(&N,&NRHS,D,E,B,&LDB,&info);
        //spbtrs_(&UPLO, &N, &KD, &NRHS, &AB[0][0], &LDAB, &B[0][0], &LDB, &info);

        std::cout << "info: "<<info << std::endl;

        std::cout << "Result: ";
        for(i=0;i<N;i++)
            std::cout << B[i] << " ";
        std::cout << std::endl;
        for(i=0;i<N;i++){
            a[i] = X1[i];
        }
        for(i=0;i<N-1;i++){
            b[i] = B[i];
            c[i] = 3*(a[i+1]-a[i])-2*B[i]-B[i+1];
            d[i] = 2*(a[i]-a[i+1])+B[i]+B[i+1];
            std::cout
                << "fA"
                << i+1
                << "[t_]:="
                << a[i]
                << "+"
                << b[i]
                << "*t+"
                << c[i]
                << "*t^2+"
                << d[i]
                << "*t^3"
                << "\n";
        }

        B[0] = 3*(X2[1]-X2[0]);
        B[1] = 3*(X2[2]-X2[0]);
        B[2] = 3*(X2[3]-X2[1]);
        B[3] = 3*(X2[3]-X2[2]);

        std::cout << "Solve: ";
        for(i=0;i<N;i++)
            std::cout << B[i] << " ";
        std::cout << std::endl;

        spttrs_(&N,&NRHS,D,E,B,&LDB,&info);

        std::cout << "Result: ";
        for(i=0;i<N;i++)
            std::cout << B[i] << " ";
        std::cout << std::endl;
        for(i=0;i<N;i++){
            a[i] = X2[i];
        }
        for(i=0;i<N-1;i++){
            b[i] = B[i];
            c[i] = 3*(a[i+1]-a[i])-2*B[i]-B[i+1];
            d[i] = 2*(a[i]-a[i+1])+B[i]+B[i+1];

            std::cout
                << "f"
                << i+1
                << "[t_]:="
                << a[i]
                << "+"
                << b[i]
                << "*t+"
                << c[i]
                << "*t^2+"
                << d[i]
                << "*t^3"
                << "\n";
        }
    }

    std::cout
        << "************ Testing Interpolator functions finished successfully!!\n";
        */
}

void InterpolatorMessage(){
    BOOST_MESSAGE("PathTestSuite");
}

InterpolatorTestSuite::InterpolatorTestSuite() :
    boost::unit_test::test_suite("InterpolatorTestSuite")
{
    add( BOOST_TEST_CASE( &InterpolatorMessage) );
    add( BOOST_TEST_CASE( &InterpolatorTest) );
}
