/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/


#include "CubicSplineInterpolator.hpp"
#include "CubicSegment.hpp"

#include <boost/numeric/ublas/io.hpp>

using namespace rw::math;
using namespace boost::numeric::ublas;
using namespace rw::interpolator;

extern "C" {
    void sptsv_(int *n, int *nrhs, float *d, float *e, float *b, int *ldb,
                long *info);
}

CubicSplineInterpolator::CubicSplineInterpolator(
         Q& qStart, 
         const std::vector<std::pair<Q, double> >& viapoints):
             _dqStart(qStart.size()),
             _dqEnd(qStart.size())
{    
    _viaPoints.push_back(std::pair<Q, double>(qStart, 0));
    typedef std::vector<std::pair<Q, double> >::const_iterator I;
    for (I it = viapoints.begin(); it != viapoints.end(); ++it)
        _viaPoints.push_back(*it);
    
    calculateSpline();
}

CubicSplineInterpolator::CubicSplineInterpolator(
        rw::math::Q& qStart, 
        const std::vector<std::pair<rw::math::Q, double> >& viapoints,
        const rw::math::Q& dqStart,
        const rw::math::Q& dqEnd):
            _dqStart(dqStart),
            _dqEnd(dqEnd)
{
    _viaPoints.push_back(std::pair<Q, double>(qStart, 0));
    typedef std::vector<std::pair<Q, double> >::const_iterator I;
    for (I it = viapoints.begin(); it != viapoints.end(); ++it)
        _viaPoints.push_back(*it);
    
    //calculateClampedSpline();
}


/*CubicSplineInterpolator::CubicSplineInterpolator(const std::vector<Q>& viaPoints,
                                                 const std::vector<double>& times)
{
    _viaPoints = viaPoints;
    _times = times;
    calculateSpline();
}*/

CubicSplineInterpolator::~CubicSplineInterpolator()
{
    for (std::vector<FunctionSegment*>::iterator it = _segments.begin(); it != _segments.end(); ++it) {
        delete (*it);
    }
}

/**
 * cubic poly: si(xj) = ai*h^3 + bi*h^2 + ci*h + di
 * where: h = xj-xi; and j=i-1 and i = [0;N]
 * 
 * ai = (Mk-Mi)/6h
 * bi = Mi/2
 * ci = (Yk-Yi)/h - h*(Mk+2*Mi)/6
 * di = Yi
 * 
 * Solve A.M = B 
 * where Y and M is vector of N-2 and A is tridiagonal Matrix of N-2 by N-2
 * 
 * A for natural spline example with N=6: 
 * 
 * 4 1 0 0
 * 1 4 1 0 
 * 0 1 4 1 
 * 0 0 1 4 
 *
 * A for clamped spline example with N=6: 
 * 
 * 4 1 0 0
 * 1 4 1 0 
 * 0 1 4 1 
 * 0 0 1 4
 */
void CubicSplineInterpolator::calculateNaturalSpline()
{
    int dim = (int)_viaPoints[0].first.size();
    int N = _viaPoints.size(); // since M1==Mn==0
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<float> B(N-2);
    std::vector<float> D(N-2);
    std::vector<float> E(N-2);
    std::vector<float> a(dim*N  );
    std::vector<float> b(dim*N-1);
    std::vector<float> c(dim*N-1);
    std::vector<float> d(dim*N-1);
    int i, j;

    for (j=0; j<dim; j++) {
        // Construct tridiagonal matrix
        for (i=0; i< (int)D.size(); i++) {
            D[i] = 4;
            E[i] = 1;
        }
        // Now construct the Y vector
        for (i=1; i<N-1; i++) {
            float h   = (float)(_viaPoints[i].second - _viaPoints[i-1].second);
            float yn0 = (float)(_viaPoints[i-1].first)[j];
            float yn1 = (float)(_viaPoints[i].first)[j];
            float yn2 = (float)(_viaPoints[i].first)[j];
            
            B[i] = (float)(6.0/(h*h) * (yn0 - 2*yn1 + yn2));
        }      
        
        // Solve for M: A.M = B
        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);
        
        // result is now in B, put it in M just for the convinience
        std::vector<float> &M = B;

        // calculate ai,bi,ci and di
        for (i=0; i < (int)M.size()-1; i++) {
            float h   = (float)(_viaPoints[i+1].second - _viaPoints[i].second);
            size_t idx = j+i*dim;
            a[idx] = (M[i+1] - M[i])/(6.0*h);
            b[idx] = M[i]/2;
            
            float y0 = (float)(_viaPoints[i].first)[j];
            float y1 = (float)(_viaPoints[i+1].first)[j];
            
            c[idx] = (y1-y0)/h - h*(M[i+1] + 2*M[i])/6.0;
            d[idx] = y0;
        }
    }

    _length = 0;
    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=d[j+i*dim];
            bb[j]=c[j+i*dim];
            bc[j]=b[j+i*dim];
            bd[j]=a[j+i*dim];
        }
        
        //Calculate the distance relative to the previous point
        double time = _viaPoints.at(i+1).second;
        if (i > 0) 
            time -= _viaPoints.at(i).second;
        
        //Is it a new segment or should be just modify an existing?
        if (i>=(int)_segments.size()) {
            CubicSegment *cseg = new CubicSegment(ba, bb, bc, bd, time);
            _segments.push_back(cseg);
        } else {
            ((CubicSegment*)_segments[i])->setParameters(ba,
                                                         bb,
                                                         bc,
                                                         bd,
                                                         time);
        }
    }
    int seg_size = _segments.size();

    // Decrease the size to match the number of via points
    for (int i = 0; i < seg_size - (N - 1); i++) {
        FunctionSegment* seg = _segments.back();
        _segments.pop_back();
        delete seg;
    }
    _length = _viaPoints.back().second-_viaPoints.front().second;
}




void CubicSplineInterpolator::calculateSpline()
{
    int dim = (int)_viaPoints[0].first.size();
    int N = _viaPoints.size();
    int NRHS = 1;
    long info = 0;
    int LDB = N;

    std::vector<float> B(N);
    std::vector<float> D(N);
    std::vector<float> E(N);
    std::vector<float> a(dim*N);
    std::vector<float> b(dim*N-1);
    std::vector<float> c(dim*N-1);
    std::vector<float> d(dim*N-1);
    int i, j;

    for (j=0; j<dim; j++) {
        for (i=0; i<N; i++) {
            D[i] = 4;
            E[i] = 1;
        }
        D[0] = 2;
        D[N-1] = 2;

        for (i=1; i<N-1; i++) {
            B[i] = (float)(3 * ((_viaPoints[i+1].first)[j] - (_viaPoints[i-1].first)[j]));
        }

        B[0] = (float)(3*((_viaPoints[1].first)[j]-(_viaPoints[0].first)[j]));
        B[N-1] = (float)(3*((_viaPoints[N-1].first)[j]-(_viaPoints[N-2].first)[j]));

        sptsv_(&N, &NRHS, &D.at(0), &E.at(0), &B.at(0), &LDB, &info);

        for (i=0; i<N; i++) {
            a[j+i*dim] = (float)(_viaPoints[i].first)[j];
        }

        for (i=0; i<N-1; i++) {
            b[j+i*dim] = B[i];
            c[j+i*dim] = 3*(a[j+(i+1)*dim]-a[j+i*dim])-2*B[i]-B[i+1];
            d[j+i*dim] = 2*(a[j+i*dim]-a[j+(i+1)*dim])+B[i]+B[i+1];
        }
    }

    _length = 0;
    Q ba(dim), bb(dim), bc(dim), bd(dim);
    for (i=0; i<N-1; i++) {
        for (j=0; j<dim; j++) {
            ba[j]=a[j+i*dim];
            bb[j]=b[j+i*dim];
            bc[j]=c[j+i*dim];
            bd[j]=d[j+i*dim];
        }
        
        //Calculate the distance relative to the previous point
        double time = _viaPoints.at(i+1).second;
        if (i > 0) 
            time -= _viaPoints.at(i).second;
        
        //Is it a new segment or should be just modify an existing?
        if (i>=(int)_segments.size()) {
            CubicSegment *cseg = new CubicSegment(ba, bb, bc, bd, time);
            _segments.push_back(cseg);
        } else {
            ((CubicSegment*)_segments[i])->setParameters(ba,
                                                         bb,
                                                         bc,
                                                         bd,
                                                         time);
        }
    }
    int seg_size = _segments.size();

    // Decrease the size to match the number of via points
    for (int i = 0; i < seg_size - (N - 1); i++) {
        FunctionSegment* seg = _segments.back();
        _segments.pop_back();
        delete seg;
    }
    _length = _viaPoints.back().second-_viaPoints.front().second;
}

Q CubicSplineInterpolator::getX(double d) const {
    double time = 0, tmptime;
    for (std::vector<FunctionSegment*>::const_iterator it = _segments.begin(); it != _segments.end(); ++it) {
        tmptime = time;
        time += (*it)->getLength();
        if (time > d) {
            return (*it)->getX(d-tmptime);
        }
    }
    return _segments.back()->getX(_segments.back()->getLength());
}

std::vector<Q> CubicSplineInterpolator::getViaPoints() const {
    std::vector<Q> result;
    typedef std::vector<std::pair<Q, double> >::const_iterator I;
    for (I it = _viaPoints.begin(); it != _viaPoints.end(); ++it) {
        result.push_back((*it).first);               
    }
    return result;
}

