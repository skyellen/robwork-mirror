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
#include "Contour2D.hpp"

#include <boost/foreach.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/math/Rotation2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Line2D.hpp>

#include "Triangulate.hpp"

#include <fstream>

using namespace rw::math;
using namespace rw::geometry;

double Contour2D::calcArea()
{
    int n = (int)_points.size();

    double area=0.0;

    for(int p=n-1,q=0; q<n; p=q++)
    {
        area += _points[p].P()(0)*_points[q].P()(1) - _points[q].P()(0)*_points[p].P()(1);
    }
    return area*0.5f;
}


void Contour2D::write(Contour2D& objC, std::string file){
    std::ofstream ostr(file.c_str());
    if (!ostr.is_open())
        RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));

    ostr << "ObjectContour \n";
    ostr << "Size " << objC.size() << "\n";
    ostr << "Center " << objC.center()(0) << " " << objC.center()(1) << "\n";
    //std::cout << "size: " << objC.size() << std::endl;
    //std::cout << "center: " << objC.center << std::endl;
    for(size_t i=0; i<objC.size(); i++){
        Contour2D::Point &point = objC[i];
        rw::math::Vector2D<> pos = point.P();
        rw::math::Vector2D<> dir = point.N();
        ostr << "Pos " << pos(0) << " " << pos(1) << "\n";
        ostr << "Dir " << dir(0) << " " << dir(1) << "\n";
    }
    ostr.close();
}

Contour2D Contour2D::read(std::string file){
    std::ifstream inp(file.c_str());
    if (!inp.is_open())
        RW_THROW("Can't read file " << rw::common::StringUtil::quote(file));
    std::string strToken;
    inp >> strToken;

    int size = 0;
    rw::math::Vector2D<> center;
    inp >> strToken >> size;
    inp >> strToken >> center(0) >> center(1);
    //std::cout << "size: " << size << std::endl;
    Contour2D objC;
    objC.center() = center;
    //std::cout << "center: " << center<< std::endl;
    objC.points().resize(size);
    for(size_t i=0; i<objC.size(); i++){
        rw::math::Vector2D<> pos, dir;

        inp >> strToken >> pos(0) >> pos(1);
        inp >> strToken >> dir(0) >> dir(1);
        objC[i] = Contour2D::Point(pos,dir);
    }
    inp.close();
    return objC;
}


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

         Vector2D<> getAverage() const{
             return _sum/_len;
         }

     private:
         const int _len;
         std::vector<Vector2D<> > _cb;
         Vector2D<> _sum;
         int _idx;
     };

}


double Contour2D::calcSequenceMoment(
    const Contour2D& contour,
    const rw::math::Vector2D<>& c,
    const int r)
{
    double mr = 0;
    //BOOST_FOREACH(const rw::math::Vector2D<>& p, contour){
    BOOST_FOREACH(const Contour2D::Point& p, contour.points()){
        const double val = (c-p.P()).norm2();
        double m = val;
        for(int i=0;i<r;i++)
            m *= val;
        mr += m;
    }
    const int N = (int)contour.size();
    return mr/N;
}

double Contour2D::calcCentralMoments(const Contour2D& contour, const rw::math::Vector2D<>& c, const int r)
{
    double m1 = calcSequenceMoment(contour,c,1);
    double ur = 0;
    BOOST_FOREACH(const Contour2D::Point& p, contour.points()){
        const double val = (c-p.P()).norm2() - m1;
        double u = val;
        for(int i=0;i<r;i++)
            u *= val;
        ur+=u;
    }
    const int N = (int)contour.size();
    return ur/N;
}

Covariance<> Contour2D::calcCovarianceMatrix(
    const Contour2D& contour, const rw::math::Vector2D<>& c)
{
    Eigen::MatrixXd covarM = Eigen::MatrixXd::Zero(2,2);
    //BOOST_FOREACH(const rw::math::Vector2D<>& q, contour){
    BOOST_FOREACH(const Contour2D::Point& cp, contour.points()){
        const Vector2D<>& q = cp.P();
        const rw::math::Vector2D<>& p = q-c;
        covarM(0,0) += p[0]*p[0];
        covarM(0,1) += p[0]*p[1];
        covarM(1,0) += p[1]*p[0];
        covarM(1,1) += p[1]*p[1];
        //covarM(j,k) += 9*c[j]*c[k] + p[j]*p[k] + q[j]*q[k] + r[j]*r[k];
    }
    const int N = (int)contour.size();
    covarM(0,0) = covarM(0,0)/N;
    covarM(0,1) = covarM(0,1)/N;
    covarM(1,0) = covarM(1,0)/N;
    covarM(1,1) = covarM(1,1)/N;
    return Covariance<>(covarM);
    //covar(j,k) = covar(j,k)/(12.0*totA)-centroid[j]*centroid[k];
}

rw::math::Vector2D<> Contour2D::calcCentroid(const Contour2D& contour)
{
    Vector2D<> c(0,0);
    //BOOST_FOREACH(const Vector2D<>& q, contour){
    BOOST_FOREACH(const Contour2D::Point& p, contour.points()){
        c += p.P();
    }
    return c/((int)contour.size());
}


Rotation2D<> Contour2D::calcOrientation(
    const Contour2D& contour, const rw::math::Vector2D<>& c)
{
    Covariance<> covar = calcCovarianceMatrix(contour, c);
    Eigen::MatrixXd covarTmp = covar.getMatrix();
    EigenDecomposition<> decomp = covar.eigenDecompose();
    decomp.sort();

    // specify x and y axis, x will be the axis with largest spred
    rw::math::Vector2D<> maxAxis( decomp.getEigenVector(1)[0], decomp.getEigenVector(1)[1] );
    rw::math::Vector2D<> minAxis( decomp.getEigenVector(0)[0], decomp.getEigenVector(0)[1]  );

    // generate the rotation matrix
    Rotation2D<> rot(normalize(maxAxis),normalize(minAxis));
    return rot;
}

/**
 * @brief extracts the local curvature around the contour point
 * defined by idx.
 */
double Contour2D::getCurvature(int idx, int pixelStep, const Contour2D& contour)
{
    Vector2D<> v2d;
    Vector2D<> center = contour.points()[idx].P();
    Vector2D<> left;
    Vector2D<> right;

    if(idx-pixelStep < 0 )
        left = contour[ contour.size()+(idx-pixelStep) ].P();
    else
        left = contour[idx-pixelStep].P();
    if(idx+pixelStep >= (int)contour.size() )
        right = contour[ (idx+pixelStep)-contour.size() ].P();
    else
        right = contour[idx+pixelStep].P();

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



rw::math::Vector2D<> Contour2D::calcNormal(
        int idx, int pixelStep, const Contour2D& contour, bool counterClock)
{
    rw::math::Vector2D<> center = contour[idx].P(), left, right;

    if(idx-pixelStep < 0 ) left = contour[ contour.size()+(idx-pixelStep) ].P();
    else left = contour[idx-pixelStep].P();

    if(idx+pixelStep >= (int)contour.size() ) right = contour[ (idx+pixelStep)-contour.size() ].P();
    else right = contour[idx+pixelStep].P();

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

void Contour2D::recalcNormal(Contour2D& contour){
    int _avgFilterLen = 4;
    MovingAverage curvAvg1(_avgFilterLen), curvAvg2(_avgFilterLen);
    for(int i = (int)contour.size()-_avgFilterLen; i<_avgFilterLen; i++){
        int sampleIdx = i;
        if(i >= (int)contour.size() )
            sampleIdx = i-(int)contour.size();
        Vector2D<> normal = Contour2D::calcNormal(sampleIdx, 6, contour, true);

        curvAvg1.addSample( normal );
        if(i>=0)
            curvAvg2.addSample( curvAvg1.getAverage() );
    }

    // initialize

    for(size_t i=0; i<contour.size(); i++){

        int sampleIdx = (int)i+_avgFilterLen;
        if(sampleIdx>= (int)contour.size())
            sampleIdx = sampleIdx-(int)contour.size();

        Vector2D<> normal = Contour2D::calcNormal(sampleIdx, 6, contour, true);
        curvAvg1.addSample( normal );
        curvAvg2.addSample( curvAvg1.getAverage() );

        contour[i].N() =  curvAvg2.getAverage() ;
    }

}

namespace {
    int calcIdx(double val, double step){
        return (int)std::floor( fabs(val)/step+0.5 );
    }
}

Contour2D::Ptr Contour2D::getOuterContour(const Contour2D& contour, double res){
    // first get min and max in the y and x axis
    const double CMAX = 10000,CMIN = -10000;
    double xmin(CMAX),xmax(CMIN),ymin(CMAX),ymax(CMIN);

    std::cout << "Get min max" << std::endl;
    for(size_t i=0; i<contour.size(); i++){
        const Vector2D<> p = contour[i].P();
        if( p(0)<xmin ) xmin = p(0);
        if( p(0)>xmax ) xmax = p(0);
        if( p(1)<ymin ) ymin = p(1);
        if( p(1)>ymax ) ymax = p(1);
    }

    std::cout << "X minmax: (" << xmin <<"," << xmax << ") Y minmax: (" << ymin << "," << ymax << ")" << std::endl;
    // now use the resolution to create a

    int xResolution = (int)(fabs(xmax-xmin)/res);
    if( xResolution<5 )
        return NULL;

    double xStep = fabs(xmax-xmin)/xResolution;

    int yResolution = (int)(fabs(ymax-ymin)/res);
    if( yResolution<5 )
        return NULL;
    double yStep = fabs(ymax-ymin)/yResolution;

    std::cout << "Step: (" << xStep << "," << yStep << ")" << std::endl;

    std::vector<double> xVmin(yResolution+1,CMAX), xVmax(yResolution+1,CMIN);
    std::vector<double> yVmin(xResolution+1,CMAX), yVmax(xResolution+1,CMIN);

    std::cout << "RunA" << std::endl;
    Vector2D<> lastPoint = contour.points().back().P();
    for(size_t i=0; i<contour.size(); i++){
        const Vector2D<> p1 = lastPoint;
        const Vector2D<> p = contour[i].P();
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
        pnts.push_back( Contour2D::Point(lastPnt,defDir) );

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
        pnts.push_back( Contour2D::Point(lastPnt,defDir) );

    }

    for(int i=startIdx;i<xVmin.size();i++){

    }

*/

    std::vector< Contour2D::Point > pnts;
    // now create all the points in the new contour
/*    std::cout << "RunB" << std::endl;
    std::vector< Contour2D::Point > pnts;
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
        pnts.push_back( Contour2D::Point(lastPnt,defDir) );

    }

    std::cout << "RunC" << std::endl;
    startIdx = calcIdx(lastPnt(0)-xmin,xStep);
    std::cout << "Startidx: " << startIdx << std::endl;
    for(int i=startIdx;i<yVmax.size();i++){
        if( calcIdx( xVmax[ calcIdx(yVmax[i]-ymin, yStep) ]-xmin, xStep) == i )
            break;

        lastPnt = Vector2D<>((i+0.5)*xStep+xmin, yVmax[i] );
        pnts.push_back( Contour2D::Point(lastPnt,defDir) );
    }


    std::cout << "RunD" << std::endl;
    startIdx = calcIdx( lastPnt(1)-ymin, yStep );
    std::cout << "Startidx: " << startIdx << std::endl;
    for(int i=startIdx;i>=0;i--){
        if( calcIdx( yVmin[ calcIdx(xVmax[i]-xmin, xStep) ]-ymin, yStep) == i )
            break;

        lastPnt = Vector2D<>( xVmax[i], (i+0.5)*yStep+ymin );
        pnts.push_back(  Contour2D::Point(lastPnt, defDir) );
    }

    std::cout << "RunE" << std::endl;
    startIdx = calcIdx(lastPnt(0)-xmin,xStep);
    std::cout << "Startidx: " << startIdx << std::endl;
    int endIdx = calcIdx(pnts.front().P()(0)-xmin,xStep);
    for(int i=startIdx;i>=endIdx;i--){
        lastPnt = Vector2D<>((i+0.5)*xStep+xmin, yVmin[i] );
        pnts.push_back(  Contour2D::Point(lastPnt,defDir) );
    }
*/
    //std::vector< Contour2D::Point > pntsRev(pnts.size());
    //for(int i=0,j=pnts.size()-1; i<pnts.size(); i++,j--){
    //    pntsRev[j] = pnts[i];
    //}

    //std::cout << "pntsIdx :" << pntsIdx << std::endl;
    //std::cout << "Nr of points in outer contour: " << pnts.size() << std::endl;
    // now the points are collected in one complete sequence and we construct the Contour2D
    return Contour2D::Ptr( new Contour2D( contour.center(), pnts));
}


namespace
{
    // Faces for either the top or bottom of the geometry.
    void contourFaces(
        const Contour2D& object,
        double z,
        const Vector3D<float>& normal,
        PlainTriMeshN1F::Ptr mesh)
    {
        std::vector<Vector2D<> > pnts;
        BOOST_FOREACH(const Contour2D::Point& pnt, object.points()) {
            pnts.push_back(
                Vector2D<>(
                    (float)pnt.P()(0),
                    (float)pnt.P()(1)));
        }

        std::vector<int> triangulation;
        Triangulate::processPoints(pnts, triangulation);

        for(size_t i=0; i<triangulation.size();i+=3){
            const Vector2D<>& p1 = pnts[ triangulation[i * 3 + 0] ];
            const Vector2D<>& p2 = pnts[ triangulation[i * 3 + 1] ];
            const Vector2D<>& p3 = pnts[ triangulation[i * 3 + 2] ];

            mesh->add(
                TriangleN1<float>(
                    Vector3D<float>((float)p1[0], (float)p1[1], (float)z),
                    Vector3D<float>((float)p2[0], (float)p2[1], (float)z),
                    Vector3D<float>((float)p3[0], (float)p3[1], (float)z),
                    normal));
        }
    }

    // The horizontal faces of the object.
    void sideFaces(
        const Contour2D& object,
        double height,
        PlainTriMeshN1F::Ptr mesh)
    {
        const int len = (int)object.points().size();
        for (int i = 1; i < len; i++) {
            const Vector2D<> p = object.points()[i - 1].P();
            const Vector2D<> q = object.points()[i].P();

            const Vector3D<float> a1((float)p(0), (float)p(1), 0);
            const Vector3D<float> a2((float)q(0), (float)q(1), 0);
            const Vector3D<float> b1((float)p(0), (float)p(1), (float)height);
            const Vector3D<float> b2((float)q(0), (float)q(1), (float)height);

            const Vector3D<float> normal = normalize(cross(a2 - a1, b1 - a1));

            mesh->add( TriangleN1<float>(a1, a2, b1, normal) );
            mesh->add( TriangleN1<float>(a2, b2, b1, normal) );
        }
    }
}

TriMesh::Ptr Contour2D::toTriMesh(double height)
{
    if (this->points().size() < 3)
        RW_THROW("No contour.");

    PlainTriMeshN1F::Ptr mesh = rw::common::ownedPtr( new PlainTriMeshN1F() );

    sideFaces(*this, height, mesh);

    if(height==0){
        // top
        contourFaces(*this, 0, Vector3D<float>(0, 0, 1), mesh);
    } else {
        // floor
        contourFaces(*this, 0, Vector3D<float>(0, 0, -1), mesh);

        // top
        contourFaces(*this, height, Vector3D<float>(0, 0, 1), mesh);
    }
    return mesh;
}


