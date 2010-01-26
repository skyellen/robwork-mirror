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


using namespace rw::trajectory;

using namespace rw::math;
using namespace boost::numeric::ublas;
using namespace rw::common;

extern "C" {
    void sptsv_(int *n, int *nrhs, float *d, float *e, float *b, int *ldb,
                long *info);

    void dptsv_(int *n, int *nrhs, double *d, double *e, double *b, int *ldb,
                long *info);

}

InterpolatorTrajectory<Q>*
	CubicSplineFactory::makeNaturalSpline(QPathPtr qpath)
{
	typedef float T;

    int dim = (*qpath)[0].size(); // (int)_viaPoints[0].first.size();
    int N = qpath->size();
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<T> B(N);
    std::vector<T> D(N);
    std::vector<T> E(N);
    std::vector<T> a(dim*N);
    std::vector<T> b(dim*N-1);
    std::vector<T> c(dim*N-1);
    std::vector<T> d(dim*N-1);

    int i, j;

    for (j=0; j<dim; j++) {
        for (i=0; i<N; i++) {
            D[i] = 4; // 2*(h(i-1)+h(i))
            E[i] = 1; // h(i)
        }
        D[0] = 4;
        D[N-1] = 4;

        for (i=1; i<N-1; i++) {
            B[i] = (3 * (((*qpath)[i+1])[j] - ((*qpath)[i-1])[j]));
        }

        B[0] = 3*(((*qpath)[1])[j]-((*qpath)[0])[j]);
        B[N-1] = 3*(((*qpath)[N-1])[j]-((*qpath)[N-2])[j]);

        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);

        for (i=0; i<N; i++) {
            a[j+i*dim] = ((*qpath)[i])[j];
        }

        for (i=0; i<N-1; i++) {
            b[j+i*dim] = B[i];
            c[j+i*dim] = 3*(a[j+(i+1)*dim]-a[j+i*dim])-2*B[i]-B[i+1];
            d[j+i*dim] = 2*(a[j+i*dim]-a[j+(i+1)*dim])+B[i]+B[i+1];
        }
    }

    InterpolatorTrajectory<Q>* traj =
			new InterpolatorTrajectory<Q>();

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }

        // the time is always 1 in this version of the spline
        double time = 1;

        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, time);
        Ptr<Interpolator<Q> > cseg = Ptr<Interpolator<Q> >( iptr );

        traj->add( cseg );
    }

	return traj;
}

InterpolatorTrajectory<rw::math::Q>*
	makeNaturalSpline(TimedQPathPtr tqpath,
	    double offset)
{
	typedef float T;

    int dim = (*tqpath)[0].getValue().size(); // (int)_viaPoints[0].first.size();
    int N = tqpath->size();
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<T> B(N);
    std::vector<T> D(N);
    std::vector<T> E(N);
    std::vector<T> H(N);
    std::vector<T> a(dim*N);
    std::vector<T> b(dim*N-1);
    std::vector<T> c(dim*N-1);
    std::vector<T> d(dim*N-1);

    int i, j;
    H[0] = (*tqpath)[0].getTime();
    for(i=1;i<N;i++){
    	H[i] = (*tqpath)[i].getTime()-(*tqpath)[i-1].getTime();
    }
    for (j=0; j<dim; j++) {
        for (i=1; i<N-1; i++) {
            D[i] = 2*(H[i-1]+H[i]); // 2*(h(i-1)+h(i))
            E[i] = H[i]; // h(i)
        }
        D[0] = 2*H[0];;
        D[N-1] = 2*H[N-1];

        for (i=1; i<N-1; i++) {
        	// 3/hi
            B[i] = 3/H[i] * (((*tqpath)[i+1].getValue()[j] - (*tqpath)[i].getValue()[j])) -
            		3/H[i-1]* (((*tqpath)[i].getValue()[j] - (*tqpath)[i-1].getValue()[j]));
        }

        B[0] = 3/H[0]*((*tqpath)[1].getValue()[j]-(*tqpath)[0].getValue()[j]);
        B[N-1] = 3/H[N-1]*((*tqpath)[N-1].getValue()[j]-(*tqpath)[N-2].getValue()[j]);

        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);

        for (i=0; i<N; i++) {
            a[j+i*dim] = (*tqpath)[i].getValue()[j];
        }

        for (i=0; i<N-1; i++) {
            b[j+i*dim] = B[i];
            c[j+i*dim] = 3*(a[j+(i+1)*dim]-a[j+i*dim])-2*B[i]-B[i+1];
            d[j+i*dim] = 2*(a[j+i*dim]-a[j+(i+1)*dim])+B[i]+B[i+1];
        }
    }

    InterpolatorTrajectory<Q>* traj =
			new InterpolatorTrajectory<Q>();

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }

        double time = H[i];

        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, time);
        Ptr<Interpolator<Q> > cseg = Ptr<Interpolator<Q> >( iptr );

        traj->add( cseg );
    }

	return traj;
}

InterpolatorTrajectory<Q>*
	CubicSplineFactory::makeClampedSpline(QPathPtr qpath,
			const rw::math::Q& dqStart,
		    const rw::math::Q& dqEnd)
{
	typedef float T;

    int dim = (*qpath)[0].size(); // (int)_viaPoints[0].first.size();
    int N = qpath->size();
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<T> B(N);
    std::vector<T> D(N);
    std::vector<T> E(N);
    std::vector<T> a(dim*N);
    std::vector<T> b(dim*N-1);
    std::vector<T> c(dim*N-1);
    std::vector<T> d(dim*N-1);

    int i, j;

    for (j=0; j<dim; j++) {
        for (i=0; i<N; i++) {
            D[i] = 4; // 2*(h(i-1)+h(i))
            E[i] = 1; // h(i)
        }
        D[0] = 2;
        D[N-1] = 2;

        for (i=1; i<N-1; i++) {
            B[i] = (3 * (((*qpath)[i+1])[j] - ((*qpath)[i-1])[j]));
        }

        B[0] = 3*(((*qpath)[1])[j]-((*qpath)[0])[j]) - 3*dqStart[j];
        B[N-1] = 3*dqEnd[j] - 3*(((*qpath)[N-1])[j]-((*qpath)[N-2])[j]);

        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);

        for (i=0; i<N; i++) {
            a[j+i*dim] = ((*qpath)[i])[j];
        }

        for (i=0; i<N-1; i++) {
            b[j+i*dim] = B[i];
            c[j+i*dim] = 3*(a[j+(i+1)*dim]-a[j+i*dim])-2*B[i]-B[i+1];
            d[j+i*dim] = 2*(a[j+i*dim]-a[j+(i+1)*dim])+B[i]+B[i+1];
        }
    }

    InterpolatorTrajectory<Q>* traj =
			new InterpolatorTrajectory<Q>();

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }

        // the time is always 1 in this version of the spline
        double time = 1;

        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, time);
        Ptr<Interpolator<Q> > cseg = Ptr<Interpolator<Q> >( iptr );

        traj->add( cseg );
    }

	return traj;
}

InterpolatorTrajectory<rw::math::Q>*
	makeClampedSpline(TimedQPathPtr tqpath,
		const rw::math::Q& dqStart,
	    const rw::math::Q& dqEnd,
	    double offset)
{
	typedef float T;

    int dim = (*tqpath)[0].getValue().size(); // (int)_viaPoints[0].first.size();
    int N = tqpath->size();
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<T> B(N);
    std::vector<T> D(N);
    std::vector<T> E(N);
    std::vector<T> H(N);
    std::vector<T> a(dim*N);
    std::vector<T> b(dim*N-1);
    std::vector<T> c(dim*N-1);
    std::vector<T> d(dim*N-1);

    int i, j;
    H[0] = (*tqpath)[0].getTime();
    for(i=1;i<N;i++){
    	H[i] = (*tqpath)[i].getTime()-(*tqpath)[i-1].getTime();
    }
    for (j=0; j<dim; j++) {
        for (i=1; i<N-1; i++) {
            D[i] = 2*(H[i-1]+H[i]); // 2*(h(i-1)+h(i))
            E[i] = H[i]; // h(i)
        }
        D[0] = 2*H[0];;
        D[N-1] = 2*H[N-1];

        for (i=1; i<N-1; i++) {
        	// 3/hi
            B[i] = 3/H[i] * (((*tqpath)[i+1].getValue()[j] - (*tqpath)[i].getValue()[j])) -
            		3/H[i-1]* (((*tqpath)[i].getValue()[j] - (*tqpath)[i-1].getValue()[j]));
        }

        B[0] = 3/H[0]*((*tqpath)[1].getValue()[j]-(*tqpath)[0].getValue()[j]) - 3*dqStart[j];
        B[N-1] = 3*dqEnd[j] - 3/H[N-1]*((*tqpath)[N-1].getValue()[j]-(*tqpath)[N-2].getValue()[j]);

        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);

        for (i=0; i<N; i++) {
            a[j+i*dim] = (*tqpath)[i].getValue()[j];
        }

        for (i=0; i<N-1; i++) {
            b[j+i*dim] = B[i];
            c[j+i*dim] = 3*(a[j+(i+1)*dim]-a[j+i*dim])-2*B[i]-B[i+1];
            d[j+i*dim] = 2*(a[j+i*dim]-a[j+(i+1)*dim])+B[i]+B[i+1];
        }
    }

    InterpolatorTrajectory<Q>* traj =
			new InterpolatorTrajectory<Q>();

    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }

        double time = H[i];

        Interpolator<Q> *iptr = new CubicSplineInterpolator<Q>(ba, bb, bc, bd, time);
        Ptr<Interpolator<Q> > cseg = Ptr<Interpolator<Q> >( iptr );

        traj->add( cseg );
    }

	return traj;
}

