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

#include "CubicSplineFactory.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/math/LinearAlgebra.hpp>

using namespace rw::trajectory;

using namespace rw::math;
using namespace rw::common;

using namespace boost::numeric::ublas;


namespace {

/* Fills solution into x. Warning: will modify c and d! */
    /**
     *
     * @param a [in] lower diag (n-1)
     * @param b [in] diagonal (n)
     * @param c [in] upper diag (n-1)
     * @param d [in] Y vector
     * @param x [out] solution
     * @param n [in] size of matrix
     */
    template<class T>
    void TridiagonalSolve (const T *a, const T *b, T *c, T *d, T *x, unsigned int n){

        /* Modify the coefficients. */
        c[0] /= b[0];   /* Division by zero risk. */
        d[0] /= b[0];   /* Division by zero would imply a singular matrix. */
        for (unsigned int i = 1; i < n; i++){
            T id = 1 / (b[i] - c[i-1] * a[i]);  /* Division by zero risk. */
            c[i] *= id;                          /* Last value calculated is redundant. */
            d[i] = (d[i] - d[i-1] * a[i]) * id;
        }

        /* Now back substitute. */
        x[n - 1] = d[n - 1];
        for (int i = n - 2; i >= 0; i--)
            x[i] = d[i] - c[i] * x[i + 1];
    }


}

InterpolatorTrajectory<Q>::Ptr
	CubicSplineFactory::makeNaturalSpline(QPathPtr qpath, double timeStep)
{
	typedef float T;
	using namespace boost::numeric;

    if(qpath->size()<2)
        RW_THROW("Path must be longer than 1!");

    const size_t dim = (*qpath)[0].size(); // the number of dimensions of the points
    const size_t N = qpath->size()-1; // we have N+1 points, which yields N splines

    ublas::vector<T> B(N+1); // make room for boundary conditions
    ublas::vector<T> D(N+1,4),DTmp(N+1); // the diagonal
    ublas::vector<T> E(N,1),ETmp(N); // the left/right to the diagonal
    ublas::vector<T> Y(N+1); // the points that the spline should intersect

    ublas::vector<T> a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);

    D[0] = 2;
    D[N] = 2;
    // ****** for each dimension we calculate the splines between via points
    for (size_t j=0; j<(size_t)dim; j++) {

        for (size_t i=0; i<(size_t)N+1; i++) {
    		Y[i] = (T)((*qpath)[i])[j];
    	}

        B[0] = 3*(Y[1]-Y[0])/*- dx_start*/;
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = 3*(Y[i+1]-Y[i-1]);
        }
        B[N] = /* dx_end*/-3*(Y[N]-Y[N-1]);

        ETmp = E;
        DTmp = D;

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Error solving tridiagonal system!");

        for (size_t i=0; i<N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<N; i++) {
            size_t idx = j+i*dim;
            b[idx] =  B[i];
            c[idx] =  3*(Y[i+1]-Y[i])-2*B[i]-B[i+1];
            d[idx] =  2*(Y[i]-Y[i+1])+B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>() );

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (size_t i=0; i<N; i++) {
        for (size_t j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }

        traj->add( ownedPtr(new CubicSplineInterpolator<Q>(ba, bb, bc, bd, timeStep) ) );
    }

    return traj;
}

InterpolatorTrajectory<rw::math::Q>::Ptr
    CubicSplineFactory::makeNaturalSpline(TimedQPathPtr tqpath)
{
    typedef float T;
    using namespace boost::numeric;

    if(tqpath->size()<2)
        RW_THROW("Path must be longer than 1!");
    size_t dim = (*tqpath)[0].getValue().size(); // the number of dimensions of the points
    size_t N = tqpath->size()-1; // we have N+1 points, which yields N splines

    ublas::vector<T> B(N+1); // make room for boundary conditions
    ublas::vector<T> D(N+1),DTmp(N+1); // the diagonal
    ublas::vector<T> E(N,1),ETmp(N); // the left/right to the diagonal
    ublas::vector<T> Y(N+1); // the points that the spline should intersect

    ublas::vector<T> a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);

    ublas::vector<T> H(N); // duration from point i to i+1
    for (size_t i=0; i<N; i++) {
        T timeI0 = (T)((*tqpath)[i]).getTime();
        T timeI1 = (T)((*tqpath)[i+1]).getTime();
        H[i] = timeI1-timeI0;
    }

    D[0] = 2*H[0];
    for (size_t i=1; i<D.size()-1; i++) {
        D[i] = 2*(H[i-1]+H[i]);
    }
    D[N] = 2*H[N-1];

    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)((*tqpath)[i]).getValue()[j];
        }

        ETmp = H;
        DTmp = D;

        B[0] = 3.0*(Y[1]-Y[0])/H[0];
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = 3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]);
        }
        B[N] = -3.0*(Y[N]-Y[N-1])/H[N-1];

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Error solving tridiagonal system!");

        for (size_t i=0; i<(size_t)N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/3.0;
            d[j+i*dim] =  (B[i+1]-B[i])/(3.0*H[i]);//   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>((*tqpath)[0].getTime()) );

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (size_t i=0; i<N; i++) {
        for (size_t j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }
        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, H[i]);
        traj->add( ownedPtr(iptr) );
    }

    return traj;
}

InterpolatorTrajectory<Q>::Ptr
	CubicSplineFactory::makeClampedSpline(QPathPtr qpath,
			const rw::math::Q& dqStart,
		    const rw::math::Q& dqEnd,
		    double timeStep)
{
    typedef float T;
    using namespace boost::numeric;

    if(qpath->size()<2)
        RW_THROW("Path must be longer than 1!");
    int dim = (*qpath)[0].size(); // the number of dimensions of the points
    int N = qpath->size()-1; // we have N+1 points, which yields N splines

    ublas::vector<T> B(N+1); // make room for boundary conditions
    ublas::vector<T> D(N+1),DTmp(N+1); // the diagonal
    ublas::vector<T> E(N,timeStep),ETmp(N); // the left/right to the diagonal
    ublas::vector<T> Y(N+1); // the points that the spline should intersect

    ublas::vector<T> a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);

    D[0] = 2*timeStep;
    for (size_t i=1; i<D.size()-1; i++) {
        D[i] = 2*(timeStep+timeStep);
    }
    D[N] = 2*timeStep;

    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)((*qpath)[i])[j];
        }

        ETmp = E;
        DTmp = D;

        B[0] = 3.0*(Y[1]-Y[0])/timeStep-3*dqStart[j];
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = 3.0*((Y[i+1]-Y[i])/timeStep - (Y[i]-Y[i-1])/timeStep);
        }
        B[N] = 3*dqEnd[j]-3.0*(Y[N]-Y[N-1])/timeStep;

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Error solving tridiagonal system!");

        for (size_t i=0; i<(size_t)N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (Y[i+1]-Y[i])/timeStep  -   timeStep*(B[i+1] + 2*B[i])/3.0;
            d[j+i*dim] =  (B[i+1]-B[i])/(3.0*timeStep);
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>() );

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (int i=0; i<N; i++) {
        for (int j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }
        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, timeStep);
        traj->add( ownedPtr(iptr) );
    }

    return traj;
}

InterpolatorTrajectory<rw::math::Q>::Ptr
CubicSplineFactory::makeClampedSpline(TimedQPathPtr tqpath,
		const rw::math::Q& dqStart,
	    const rw::math::Q& dqEnd)
{
    typedef float T;
    using namespace boost::numeric;

    if(tqpath->size()<2)
        RW_THROW("Path must be longer than 1!");

    size_t dim = (*tqpath)[0].getValue().size(); // the number of dimensions of the points
    size_t N = tqpath->size()-1; // we have N+1 points, which yields N splines

    ublas::vector<T> B(N+1); // make room for boundary conditions
    ublas::vector<T> D(N+1),DTmp(N+1); // the diagonal
    ublas::vector<T> E(N,1),ETmp(N); // the left/right to the diagonal
    ublas::vector<T> Y(N+1); // the points that the spline should intersect
    ublas::vector<T> a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);
    ublas::vector<T> H(N); // duration from point i to i+1

    for (size_t i=0; i<N; i++) {
        T timeI0 = (T)((*tqpath)[i]).getTime();
        T timeI1 = (T)((*tqpath)[i+1]).getTime();
        H[i] = timeI1-timeI0;
    }

    D[0] = 2*H[0];
    for (size_t i=1; i<D.size()-1; i++) {
        D[i] = 2*(H[i-1]+H[i]);
    }
    D[N] = 2*H[N-1];

    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)((*tqpath)[i]).getValue()[j];
        }

        ETmp = H;
        DTmp = D;

        B[0] = 3.0*(Y[1]-Y[0])/H[0]-3*dqStart[j];
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = 3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]);
        }
        B[N] = 3*dqEnd[j]-3.0*(Y[N]-Y[N-1])/H[N-1];

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Errorsolving tridiagonal system!");

        for (size_t i=0; i<N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/3.0;
            d[j+i*dim] =  (B[i+1]-B[i])/(3.0*H[i]);//   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>((*tqpath)[0].getTime()) );

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (size_t i=0; i<N; i++) {
        for (size_t j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }
        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, H[i]);
        traj->add( ownedPtr(iptr) );
    }

    return traj;
}

