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
#include "CubicSplineInterpolator.hpp"

#include <rw/common/Ptr.hpp>

#ifdef RW_USE_UBLAS_LAPACK
#include <rw/math/LinearAlgebra.hpp>
#endif

#include <Eigen/Sparse>


#if EIGEN_VERSION_AT_LEAST(3,1,0)
#include <Eigen/SparseCholesky>
#endif

using namespace rw::trajectory;

using namespace rw::math;
using namespace rw::common;


InterpolatorTrajectory<Q>::Ptr CubicSplineFactory::makeNaturalSpline(QPath::Ptr qpath, double timeStep)
{
	std::vector<double> times;
	for (size_t i = 0; i<qpath->size(); i++) {
		times.push_back(i*timeStep);
	}
	return makeNaturalSpline(*qpath, times);

}



InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeNaturalSpline(TimedQPath::Ptr tqpath)
{
	QPath path;
	std::vector<double> times;
	BOOST_FOREACH(const TimedQ& tq, *tqpath) {
		path.push_back(tq.getValue());
		times.push_back(tq.getTime());
	}

	return makeNaturalSpline(path, times);
}

InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeNaturalSpline(const QPath& qpath, const std::vector<double>& times) 
{
    typedef float T;
    using namespace boost::numeric;

    if(qpath.size()<2)
        RW_THROW("Path must be longer than 1!");
	if (qpath.size() != times.size()) 
		RW_THROW("Length of path and times need to be equal");

    size_t dim = (qpath)[0].size(); // the number of dimensions of the points
    size_t N = qpath.size()-1; // we have N+1 points, which yields N splines

	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	//typedef Eigen::Matrix<T, Eigen::Dynamic, 1, 1> Matrix;


    Vector B(N+1); // make room for boundary conditions

    Vector Y(N+1); // the points that the spline should intersect

    Vector a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);

    Vector H(N); // duration from point i to i+1
    for (size_t i=0; i<N; i++) {
        //T timeI0 = (T)((*tqpath)[i]).getTime();
        //T timeI1 = (T)((*tqpath)[i+1]).getTime();
        //H[i] = timeI1-timeI0;
		H[i] = (float)(times[i+1] - times[i]);
    }



#if EIGEN_VERSION_AT_LEAST(3,1,0)
	Eigen::SparseMatrix<T> A((int)N+1, (int)N+1);
    //D[0] = 2*H[0];
	A.insert(0,0) = 2*H[0];
	A.insert(0,1) = H[0];
	A.insert(1,0) = H[0];
    for (size_t i=1; i<N; i++) {
		//D[i] = 2*(H[i-1]+H[i]);
		int ei = (int) i;
		A.insert(ei,ei) = 2*(H[i-1]+H[i]); 
		A.insert(ei,ei+1) = H[i];
		A.insert(ei+1,ei) = H[i];
    }
    //D[N] = 2*H[N-1];
	A.insert((int)N, (int)N) = 2*H[N-1];

	Eigen::SimplicialLLT<Eigen::SparseMatrix<T> > solver;
	solver.compute(A);


#else
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero((int)N+1, (int)N+1);	
	A(0,0) = 2*H[0];
	A(0,1) = H[0];
	A(1,0) = H[0];
    for (size_t i=1; i<N; i++) {
		//D[i] = 2*(H[i-1]+H[i]);
		int ei = (int) i;
		A(ei,ei) = 2*(H[i-1]+H[i]); 
		A(ei,ei+1) = H[i];
		A(ei+1,ei) = H[i];
    }
    //D[N] = 2*H[N-1];
	A((int)N, (int)N) = 2*H[N-1];
	Eigen::LLT<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > solver;
	solver.compute(A);
#endif


    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)(qpath[i])[j];
        }
		
		B[0] = (T)(3.0*(Y[1]-Y[0])/H[0]);
        for (size_t i=1; i<N; i++) {
            B[i] = (T)(3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]));
        }
        B[N] = (T)(-3.0*(Y[N]-Y[N-1])/H[N-1]);

        B = solver.solve(B);

        for (size_t i=0; i<(size_t)N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/(T)3.0;
            d[j+i*dim] =  (B[i+1]-B[i])/((T)3.0*H[i]);//   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>(times[0]) );

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


InterpolatorTrajectory<Q>::Ptr CubicSplineFactory::makeClampedSpline(QPath::Ptr qpath,
			const rw::math::Q& dqStart,
		    const rw::math::Q& dqEnd,
		    double timeStep)
{
	std::vector<double> times;
	for (size_t i = 0; i<qpath->size(); i++) {
		times.push_back(i*timeStep);
	}
	return makeClampedSpline(*qpath, times, dqStart, dqEnd);

}

InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeClampedSpline(TimedQPath::Ptr tqpath,
		const rw::math::Q& dqStart,
	    const rw::math::Q& dqEnd)

{
	QPath path;
	std::vector<double> times;
	BOOST_FOREACH(const TimedQ& tq, *tqpath) {
		path.push_back(tq.getValue());
		times.push_back(tq.getTime());
	}

	return makeClampedSpline(path, times, dqStart, dqEnd);
}


InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeClampedSpline(const QPath& qpath, const std::vector<double>& times, const rw::math::Q& dqStart, const rw::math::Q& dqEnd)
{
    typedef float T;
    
	typedef Eigen::Matrix<T, Eigen::Dynamic, 1> Vector;
	//typedef Eigen::Matrix<T, Eigen::Dynamic, 1, 1> Matrix;
	
    if(qpath.size()<2)
        RW_THROW("Path must be longer than 1!");

	if (qpath.size() != times.size())
		RW_THROW("Length of path and times need to match");

    size_t dim = (qpath)[0].size(); // the number of dimensions of the points
    size_t N = qpath.size()-1; // we have N+1 points, which yields N splines

    Vector B(N+1); // make room for boundary conditions
    Vector Y(N+1); // the points that the spline should intersect
    Vector a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);
    Vector H(N); // duration from point i to i+1

    for (size_t i=0; i<N; i++) {
        H[i] = (float)(times[i+1]-times[i]);
    }

#if EIGEN_VERSION_AT_LEAST(3,1,0)
	Eigen::SparseMatrix<T> A((int)N+1, (int)N+1);

    //D[0] = 2*H[0];
	A.insert(0,0) = 2*H[0];
	A.insert(0,1) = H[0];
	A.insert(1,0) = H[0];
    for (size_t i=1; i<N; i++) { 
        //D[i] = 2*(H[i-1]+H[i]);
		int ei = (int) i;
		A.insert(ei,ei) = 2*(H[i-1]+H[i]);
		A.insert(ei+1,ei) = H[i];
		A.insert(ei,ei+1) = H[i];
    }
    //D[N] = 2*H[N-1];
	A.insert((int)N, (int)N) = 2*H[N-1];

	Eigen::SimplicialLLT<Eigen::SparseMatrix<T> > solver;
	solver.compute(A);
#else
	Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> A = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero((int)N+1, (int)N+1);	

    //D[0] = 2*H[0];
	A(0,0) = 2*H[0];
	A(0,1) = H[0];
	A(1,0) = H[0];
    for (size_t i=1; i<N; i++) {
        //D[i] = 2*(H[i-1]+H[i]);
		int ei = (int) i;
		A(ei,ei) = 2*(H[i-1]+H[i]);
		A(ei+1,ei) = H[i];
		A(ei,ei+1) = H[i];
    }
    //D[N] = 2*H[N-1];
	A((int)N, (int)N) = 2*H[N-1];

	Eigen::LLT<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> > solver;
	solver.compute(A);

#endif


    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)(qpath[i])[j];
        }


        B[0] = (T)(3.0*(Y[1]-Y[0])/H[0]-3*dqStart[j]);
        for (size_t i=1; i<(std::size_t)B.size()-1; i++) {
            B[i] = (T)(3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]));
        }
        B[N] = (T)(3.0*dqEnd[j]-3.0*(Y[N]-Y[N-1])/H[N-1]);


		B = solver.solve(B);
        // solution will be available in B
        //if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
        //    RW_THROW("Errorsolving tridiagonal system!");

        for (size_t i=0; i<N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (T)((Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/3.0);
            d[j+i*dim] =  (T)((B[i+1]-B[i])/(3.0*H[i]));//   +B[i]+B[i+1];
        }
    }

    // ************** now create the actual trajectory from the calcualted parameters
    InterpolatorTrajectory<Q>::Ptr traj = ownedPtr( new InterpolatorTrajectory<Q>(times[0]) );

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

#ifdef RW_USE_UBLAS_LAPACK

using namespace boost::numeric::ublas;


InterpolatorTrajectory<Q>::Ptr CubicSplineFactory::makeNaturalSplineLapack(QPath::Ptr qpath, double timeStep)
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

InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeNaturalSplineLapack(TimedQPath::Ptr tqpath)
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

        B[0] = (T)(3.0*(Y[1]-Y[0])/H[0]);
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = (T)(3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]));
        }
        B[N] = (T)(-3.0*(Y[N]-Y[N-1])/H[N-1]);

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Error solving tridiagonal system!");

        for (size_t i=0; i<(size_t)N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/(T)3.0;
            d[j+i*dim] =  (B[i+1]-B[i])/((T)3.0*H[i]);//   +B[i]+B[i+1];
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


InterpolatorTrajectory<Q>::Ptr CubicSplineFactory::makeClampedSplineLapack(QPath::Ptr qpath,
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
    ublas::vector<T> E(N,(T)timeStep),ETmp(N); // the left/right to the diagonal
    ublas::vector<T> Y(N+1); // the points that the spline should intersect

    ublas::vector<T> a(dim*(N+1)), b(dim*N), c(dim*N), d(dim*N);

    D[0] = (T)(2*timeStep);
    for (size_t i=1; i<D.size()-1; i++) {
        D[i] = (T)(2*(timeStep+timeStep));
    }
    D[N] = (T)(2*timeStep);

    for (size_t j=0; j<(size_t)dim; j++) {
        for (size_t i=0; i<(size_t)N+1; i++) {
            Y[i] = (T)((*qpath)[i])[j];
        }

        ETmp = E;
        DTmp = D;

        B[0] = (T)(3.0*(Y[1]-Y[0])/timeStep-3*dqStart[j]);
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = (T)(3.0*((Y[i+1]-Y[i])/timeStep - (Y[i]-Y[i-1])/timeStep));
        }
        B[N] = (T)(3*dqEnd[j]-3.0*(Y[N]-Y[N-1])/timeStep);

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Error solving tridiagonal system!");

        for (size_t i=0; i<(size_t)N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] = (T)( (Y[i+1]-Y[i])/timeStep  -   timeStep*(B[i+1] + 2*B[i])/3.0);
            d[j+i*dim] = (T)( (B[i+1]-B[i])/(3.0*timeStep) );
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

InterpolatorTrajectory<rw::math::Q>::Ptr CubicSplineFactory::makeClampedSplineLapack(TimedQPath::Ptr tqpath,
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

        B[0] = (T)(3.0*(Y[1]-Y[0])/H[0]-3*dqStart[j]);
        for (size_t i=1; i<B.size()-1; i++) {
            B[i] = (T)(3.0*((Y[i+1]-Y[i])/H[i] - (Y[i]-Y[i-1])/H[i-1]));
        }
        B[N] = (T)(3.0*dqEnd[j]-3.0*(Y[N]-Y[N-1])/H[N-1]);

        // solution will be available in B
        if( !LinearAlgebra::triDiagonalSolve<T>(DTmp, ETmp, B) )
            RW_THROW("Errorsolving tridiagonal system!");

        for (size_t i=0; i<N+1; i++) {
            a[i*dim+j] = Y[i];
        }

        for (size_t i=0; i<(size_t)N; i++) {
            c[j+i*dim] = B[i];
            b[j+i*dim] =  (T)((Y[i+1]-Y[i])/H[i]  -   H[i]*(B[i+1] + 2*B[i])/3.0);
            d[j+i*dim] =  (T)((B[i+1]-B[i])/(3.0*H[i]));//   +B[i]+B[i+1];
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



#endif
