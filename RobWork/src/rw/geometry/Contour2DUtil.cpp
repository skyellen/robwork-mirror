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
#include "Contour2DUtil.hpp"

#include <boost/foreach.hpp>

#include <rw/math/Constants.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rw/math/Line2D.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::geometry;

namespace {
    /**
      * @brief
      */
     struct MovingAverage {
     public:
         MovingAverage(int len):
             _len(len),_cb(_len,Vector2D<>(0.0,0.0)),_sum(0.0,0.0),_idx(0)
         {}

         void addSample(Vector2D<> sample){
             _sum -= _cb[_idx];
             _sum += sample;
             _cb[_idx] = sample;
             _idx++;
             if(_idx == _len )
                 _idx = 0;
         }

         Vector2D<> getAverage(){
             return _sum/_len;
         }

     private:
         const int _len;
         std::vector<Vector2D<> > _cb;
         Vector2D<> _sum;
         int _idx;
     };

}


double Contour2DUtil::calcSequenceMoment(
    const Contour2D& contour,
    const rw::math::Vector2D<>& c,
    const int r)
{
    double mr = 0;
    //BOOST_FOREACH(const rw::math::Vector2D<>& p, contour){
    BOOST_FOREACH(const ContourPoint& p, contour.contour){
        const double val = (c-p.getPosition()).norm2();
        double m = val;
        for(int i=0;i<r;i++)
            m *= val;
        mr += m;
    }
    const int N = contour.size();
    return mr/N;
}

double Contour2DUtil::calcCentralMoments(const Contour2D& contour, const rw::math::Vector2D<>& c, const int r)
{
    double m1 = calcSequenceMoment(contour,c,1);
    double ur = 0;
    BOOST_FOREACH(const ContourPoint& p, contour.contour){
        const double val = (c-p.getPosition()).norm2() - m1;
        double u = val;
        for(int i=0;i<r;i++)
            u *= val;
        ur+=u;
    }
    const int N = contour.size();
    return ur/N;
}

Contour2DUtil::CovarMatrix2D Contour2DUtil::calcCovarianceMatrix(
    const Contour2D& contour, const rw::math::Vector2D<>& c)
{
    CovarMatrix2D covar(ublas::zero_matrix<double>(2,2));
    //BOOST_FOREACH(const rw::math::Vector2D<>& q, contour){
    BOOST_FOREACH(const ContourPoint& cp, contour.contour){
        const Vector2D<>& q = cp.getPosition();
        const rw::math::Vector2D<>& p = q-c;
        covar(0,0) += p[0]*p[0];
        covar(0,1) += p[0]*p[1];
        covar(1,0) += p[1]*p[0];
        covar(1,1) += p[1]*p[1];
        //covar(j,k) += 9*c[j]*c[k] + p[j]*p[k] + q[j]*q[k] + r[j]*r[k];
    }
    const int N = contour.size();
    covar(0,0) = covar(0,0)/N;
    covar(0,1) = covar(0,1)/N;
    covar(1,0) = covar(1,0)/N;
    covar(1,1) = covar(1,1)/N;
    return covar;
    //covar(j,k) = covar(j,k)/(12.0*totA)-centroid[j]*centroid[k];
}

rw::math::Vector2D<> Contour2DUtil::calcCentroid(const Contour2D& contour)
{
    Vector2D<> c(0,0);
    //BOOST_FOREACH(const Vector2D<>& q, contour){
    BOOST_FOREACH(const ContourPoint& p, contour.contour){
        c += p.getPosition();
    }
    return c/contour.size();
}


Rotation2D<> Contour2DUtil::calcOrientation(
    const Contour2D& contour, const rw::math::Vector2D<>& c)
{
    CovarMatrix2D covar = calcCovarianceMatrix(contour, c);
    ublas::matrix<double> covarTmp = covar;
    typedef std::pair<ublas::matrix<double>,ublas::vector<double> > ResultType;
    ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covarTmp );
    covar = covarTmp;

    // create the rotationmatrix from the normalized eigenvectors
    // find max principal axis
    size_t maxEigIdx=0, minEigIdx=1;
    if(res.second(maxEigIdx)<res.second(minEigIdx)){
        std::swap(maxEigIdx,minEigIdx);
    }
    //std::cout << "EIGEN values: " << res.second(0) << " " << res.second(1) << std::endl;
    // specify x and y axis, x will be the axis with largest spred
    rw::math::Vector2D<> maxAxis( res.first(maxEigIdx,0), res.first(maxEigIdx,1) );
    rw::math::Vector2D<> minAxis( res.first(minEigIdx,0), res.first(minEigIdx,1) );

    // generate the rotation matrix
    //std::cout << "Max Axis: " << maxAxis << std::endl;
    //std::cout << "Min Axis: " << minAxis << std::endl;
    Rotation2D<> rot(normalize(maxAxis),normalize(minAxis));
    return rot;
}

/**
 * @brief extracts the local curvature around the contour point
 * defined by idx.
 */
double Contour2DUtil::getCurvature(int idx, int pixelStep, const Contour2D& contour)
{
    Vector2D<> v2d;
    Vector2D<> center = contour.contour[idx].getPosition();
    Vector2D<> left;
    Vector2D<> right;

    if(idx-pixelStep < 0 )
        left = contour[ contour.size()+(idx-pixelStep) ].getPosition();
    else
        left = contour[idx-pixelStep].getPosition();
    if(idx+pixelStep >= (int)contour.size() )
        right = contour[ (idx+pixelStep)-contour.size() ].getPosition();
    else
        right = contour[idx+pixelStep].getPosition();

    // now calculate the angle between line(right,center) and line(left,center)
    Vector2D<> l1 = center - right;
    Vector2D<> l2 = left - center;
    double l1len = l1.norm2();
    double l2len = l2.norm2();
    //std::cout << "l1: " << l1 << std::endl;
    //std::cout << "l2: " << l2 << std::endl;
    if(l1len<0.00000001)
        return 0;
    if(l2len<0.00000001)
        return 0;
    double dotangle = dot(l1,l2)/(l1len*l2len);
    if( fabs(1-dotangle)<0.0000001 )
        return 0;
    //std::cout << "angle: " << acos(dotangle) << std::endl;
    return fabs( acos(dotangle) );
}



rw::math::Vector2D<> Contour2DUtil::calcNormal(
        int idx, int pixelStep, const Contour2D& contour, bool counterClock)
{
    rw::math::Vector2D<> center = contour[idx].getPosition(), left, right;

    if(idx-pixelStep < 0 ) left = contour[ contour.size()+(idx-pixelStep) ].getPosition();
    else left = contour[idx-pixelStep].getPosition();

    if(idx+pixelStep >= (int)contour.size() ) right = contour[ (idx+pixelStep)-contour.size() ].getPosition();
    else right = contour[idx+pixelStep].getPosition();

    if( !counterClock )
        std::swap(left,right);
    // now calculate the normals of line(right,center) and line(left,center)
    Line2D l1(center,right), l3(left,right), l2(left,center);
    Vector2D<> v2d;
    Vector2D<> n1 = l1.calcUnitNormal()*l1.getLength();
    Vector2D<> n2 = l2.calcUnitNormal()*l2.getLength();
    Vector2D<> n3 = l3.calcUnitNormal()*l3.getLength();
    Vector2D<> nAvg = (n1+n2+n3)/(l1.getLength()+l2.getLength()+l3.getLength());
    return nAvg/nAvg.norm2();
}

void Contour2DUtil::recalcNormal(Contour2D& contour){
    int _avgFilterLen = 4;
    MovingAverage curvAvg1(_avgFilterLen), curvAvg2(_avgFilterLen);
    for(int i = contour.size()-_avgFilterLen; i<_avgFilterLen; i++){
        int sampleIdx = i;
        if(i >= (int)contour.size() )
            sampleIdx = i-contour.size();
        Vector2D<> normal = Contour2DUtil::calcNormal(sampleIdx, 6, contour, true);

        curvAvg1.addSample( normal );
        if(i>=0)
            curvAvg2.addSample( curvAvg1.getAverage() );
    }

    // initialize

    for(size_t i=0; i<contour.size(); i++){

        int sampleIdx = i+_avgFilterLen;
        if(sampleIdx>= (int)contour.size())
            sampleIdx = sampleIdx-contour.size();

        Vector2D<> normal = Contour2DUtil::calcNormal(sampleIdx, 6, contour, true);
        curvAvg1.addSample( normal );
        curvAvg2.addSample( curvAvg1.getAverage() );

        contour[i].setDirection( curvAvg2.getAverage() );
    }

}

namespace {
    int calcIdx(double val, double step){
        return (int)std::floor( fabs(val)/step+0.5 );
    }
}

boost::optional<Contour2D> Contour2DUtil::getOuterContour(const Contour2D& contour, double res){
    // first get min and max in the y and x axis
    const double CMAX = 10000,CMIN = -10000;
    double xmin(CMAX),xmax(CMIN),ymin(CMAX),ymax(CMIN);

    std::cout << "Get min max" << std::endl;
    for(size_t i=0; i<contour.size(); i++){
        const Vector2D<> p = contour[i].getPosition();
        if( p(0)<xmin ) xmin = p(0);
        if( p(0)>xmax ) xmax = p(0);
        if( p(1)<ymin ) ymin = p(1);
        if( p(1)>ymax ) ymax = p(1);
    }

    std::cout << "X minmax: (" << xmin <<"," << xmax << ") Y minmax: (" << ymin << "," << ymax << ")" << std::endl;
    // now use the resolution to create a

    int xResolution = (int)(fabs(xmax-xmin)/res);
    if( xResolution<5 )
        return boost::optional<Contour2D>();
    double xStep = fabs(xmax-xmin)/xResolution;

    int yResolution = (int)(fabs(ymax-ymin)/res);
    if( yResolution<5 )
        return boost::optional<Contour2D>();
    double yStep = fabs(ymax-ymin)/yResolution;

    std::cout << "Step: (" << xStep << "," << yStep << ")" << std::endl;

    std::vector<double> xVmin(yResolution+1,CMAX), xVmax(yResolution+1,CMIN);
    std::vector<double> yVmin(xResolution+1,CMAX), yVmax(xResolution+1,CMIN);

    std::cout << "RunA" << std::endl;
    Vector2D<> lastPoint = contour.contour.back().getPosition();
    for(size_t i=0; i<contour.size(); i++){
        const Vector2D<> p1 = lastPoint;
        const Vector2D<> p = contour[i].getPosition();
        // we extrapolate the line such that we make sure all min max rows are filled

        int yIdxFrom = calcIdx( p1(1)-ymin, yStep );
        int yIdxTo = calcIdx( p(1)-ymin, yStep);

        int mYIdx = yIdxTo-yIdxFrom;
        for(int j=0;j<=abs(mYIdx);j++){
            int yIdx = yIdxFrom;
            if(mYIdx<0) yIdx -= j;
            else yIdx += j;

            double val = p1(0);
            if(mYIdx!=0) val += (p(0)-p1(0))*j*1.0/mYIdx;
            xVmin[yIdx] = std::min( val, xVmin[yIdx] );
            xVmax[yIdx] = std::max( val, xVmax[yIdx] );
        }


        int xIdxFrom = calcIdx(p1(0)-xmin,xStep);
        int xIdxTo   = calcIdx( p(0)-xmin,xStep);

        std::cout << "Idx : " << xIdxFrom << " -> " << xIdxTo << " " ;
        int mXIdx = xIdxTo-xIdxFrom;
        for(int j=0;j<=abs(mXIdx);j++){
            int xIdx = xIdxFrom;
            if(mXIdx<0) xIdx -= j;
            else xIdx += j;
            std::cout << xIdx << " ";
            double val = p1(1);
            if(mXIdx!=0) val += (p(1)-p1(1))*j*1.0/mXIdx;
            yVmin[xIdx] = std::min(val,yVmin[xIdx]);
            yVmax[xIdx] = std::max(val,yVmax[xIdx]);
        }
        std::cout <<  std::endl;
        lastPoint = p;
    }
/*
    Vector2D<> defDir(0,0),lastPnt(0,0);
    int startIdx = 0;
    for(int i=startIdx;i<xVmin.size();i++){
        // check if cell in next axis equals this cell
        if( calcIdx( yVmax[ calcIdx(xVmin[i]-xmin, xStep) ]-ymin, yStep) == i )
            break;

        lastPnt = Vector2D<>( xVmin[i], (i+0.5)*yStep+ymin );
        std::cout << lastPnt << std::endl;
        pnts.push_back( ContourPoint(lastPnt,defDir) );

    }
    //start north-west corner
    int xIdx=0,yIdx=getNorthIdx(0);
    while( getEastIdx(xIdx)!=yIdx ){
        double y = getNorth(++xIdx);
        while( yIdx<calcIdx(y,yStep) ){
            double evalue = getWest(yIdx);
            y++;
        }
        lastPnt = Vector2D<>( xVmin[i], (i+0.5)*yStep+ymin );
        std::cout << lastPnt << std::endl;
        pnts.push_back( ContourPoint(lastPnt,defDir) );

    }

    for(int i=startIdx;i<xVmin.size();i++){

    }

*/

    std::vector< ContourPoint > pnts;
    // now create all the points in the new contour
/*    std::cout << "RunB" << std::endl;
    std::vector< ContourPoint > pnts;
    //int pntsIdx = pnts.size()-1;
    int startIdx = 0;
    for(startIdx=0;startIdx<xVmin.size();startIdx++){
        int i=startIdx;
        // continue until we find a match and the correct start idx
        if( calcIdx( xVmin[ calcIdx(yVmin[i]-ymin, yStep) ]-xmin, xStep) != i )
            break;
    }



    Vector2D<> defDir(0,0),lastPnt(0,0);
    for(int i=startIdx;i<xVmin.size();i++){
        //std::cout << xVminmax[i] << std::endl;

        // check if cell in next axis equals this cell
        if( calcIdx( yVmax[ calcIdx(xVmin[i]-xmin, xStep) ]-ymin, yStep) == i )
            break;

        lastPnt = Vector2D<>( xVmin[i], (i+0.5)*yStep+ymin );
        std::cout << lastPnt << std::endl;
        pnts.push_back( ContourPoint(lastPnt,defDir) );

    }

    std::cout << "RunC" << std::endl;
    startIdx = calcIdx(lastPnt(0)-xmin,xStep);
    std::cout << "Startidx: " << startIdx << std::endl;
    for(int i=startIdx;i<yVmax.size();i++){
        if( calcIdx( xVmax[ calcIdx(yVmax[i]-ymin, yStep) ]-xmin, xStep) == i )
            break;

        lastPnt = Vector2D<>((i+0.5)*xStep+xmin, yVmax[i] );
        pnts.push_back( ContourPoint(lastPnt,defDir) );
    }


    std::cout << "RunD" << std::endl;
    startIdx = calcIdx( lastPnt(1)-ymin, yStep );
    std::cout << "Startidx: " << startIdx << std::endl;
    for(int i=startIdx;i>=0;i--){
        if( calcIdx( yVmin[ calcIdx(xVmax[i]-xmin, xStep) ]-ymin, yStep) == i )
            break;

        lastPnt = Vector2D<>( xVmax[i], (i+0.5)*yStep+ymin );
        pnts.push_back(  ContourPoint(lastPnt, defDir) );
    }

    std::cout << "RunE" << std::endl;
    startIdx = calcIdx(lastPnt(0)-xmin,xStep);
    std::cout << "Startidx: " << startIdx << std::endl;
    int endIdx = calcIdx(pnts.front().getPosition()(0)-xmin,xStep);
    for(int i=startIdx;i>=endIdx;i--){
        lastPnt = Vector2D<>((i+0.5)*xStep+xmin, yVmin[i] );
        pnts.push_back(  ContourPoint(lastPnt,defDir) );
    }
*/
    //std::vector< ContourPoint > pntsRev(pnts.size());
    //for(int i=0,j=pnts.size()-1; i<pnts.size(); i++,j--){
    //    pntsRev[j] = pnts[i];
    //}

    //std::cout << "pntsIdx :" << pntsIdx << std::endl;
    //std::cout << "Nr of points in outer contour: " << pnts.size() << std::endl;
    // now the points are collected in one complete sequence and we construct the Contour2D
    return boost::optional<Contour2D>(Contour2D( contour.center, pnts));
}

