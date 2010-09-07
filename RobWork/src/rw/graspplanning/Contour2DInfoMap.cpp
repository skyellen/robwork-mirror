#include "Contour2DInfoMap.hpp"

#include <rw/math/Vector2D.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Math.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Line2D.hpp>

#include <rw/geometry/Contour2D.hpp>
#include <rw/geometry/Contour2DUtil.hpp>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::graspplanning;
using namespace rw::geometry;
using namespace rw::sensor;

#define PIXEL_STEP_SIZE 8

//#define RW_DEBUG(str)
#define RW_DEBUG(str) std::cout << str << std::endl

namespace {

    double clampAngle(double angle){
        while(angle<0) angle += 2*Pi;
        while(angle>2*Pi) angle -= 2*Pi;
        return angle;
    }

    /**
     * @brief
     */
    struct MovingAverage {
    public:
        MovingAverage(int len):
            _len(len),_cb(_len,0.0),_sum(0.0),_idx(0)
        {}

        void addSample(double sample){
            _sum -= _cb[_idx];
            _sum += sample;
            _cb[_idx] = sample;
            _idx++;
            if(_idx == _len )
                _idx = 0;
        }

        double getAverage(){
            return _sum/_len;
        }

    private:
        const int _len;
        std::vector<double> _cb;
        double _sum;
        int _idx;

    };


}

Contour2DInfoMap::Contour2DInfoMap(int res):
    _res(res),
    _resStep(2*Pi/res),
    _resStepInv( 1.0/_resStep)
{
}

void Contour2DInfoMap::reset(const Contour2D& contour)
{
    const int contourSize = contour.size();
    int _avgFilterLen = 8;
    if( contourSize<2*_avgFilterLen )
        RW_THROW("Contour is too small");
    RW_DEBUG("--- Resetting INFO map");
    RW_DEBUG("- contour size: " << contourSize);

    _avgCurvature = 0;
    _minCurvature = 1000;
    _maxCurvature = 0;

    _contour = &contour;
    _normalToContactsMap.clear();
    _normalToContactsMap.resize(_res+1);
    _contacts.clear();
    _contacts.resize(contourSize);

    // create the moving average filters
    MovingAverage curvAvg1(_avgFilterLen),curvAvg2(_avgFilterLen);

    // initialize the average filters
    for(int idx=contourSize-_avgFilterLen; idx<contourSize+_avgFilterLen; idx++){
        int sampleIdx = idx;
        if(idx>=contourSize)
            sampleIdx = idx-contourSize;
        double curvature = Contour2DUtil::getCurvature(sampleIdx, PIXEL_STEP_SIZE, contour);
        RW_DEBUG("pre Curvature: " << curvature);
        curvAvg1.addSample( fabs(curvature) );
        if(idx>=0)
            curvAvg2.addSample( curvAvg1.getAverage() );
    }
    // main loop
    for(int idx=0; idx<contourSize; idx++){
        // calculate normal and curvature info
        //const Vector2D<> normal = Contour2DUtil::calcNormal(idx, PIXEL_STEP_SIZE, contour);
        const double curvature = Contour2DUtil::getCurvature(idx, PIXEL_STEP_SIZE, contour);

        // update the average filter values
        int sampleIdx = idx+_avgFilterLen;
        if(sampleIdx>=contourSize)
            sampleIdx = sampleIdx-contourSize;
        double sampleCurv =  Contour2DUtil::getCurvature(sampleIdx, PIXEL_STEP_SIZE, contour);
        curvAvg1.addSample( fabs(sampleCurv) );
        curvAvg2.addSample( curvAvg1.getAverage() );

        // now insert the calculated values into the contact struct
        Contact2D &c = _contacts[idx];
        c.p = contour.points()[idx].P();
        //c.n = normal;
        c.n = contour.points()[idx].N();
        c.curvature = curvature;
        c.avgCurvature = Math::sqr(curvAvg2.getAverage());

        if ( _minCurvature>c.avgCurvature )
            _minCurvature = c.avgCurvature;
        if ( _maxCurvature<c.avgCurvature )
            _maxCurvature = c.avgCurvature;
        _avgCurvature +=  c.avgCurvature;

        // use angle of the normal relative to x-axis to compute index
        double nAngle = clampAngle( angle(Vector2D<>(1,0),c.n) );
        RW_DEBUG("Angle       : " << angle(Vector2D<>(1,0),c.n));
        RW_DEBUG("ClampedAngle: " << nAngle);
        /*
        double nAngle = atan2(c.n(1),c.n(0));
        if(nAngle<0)
            nAngle +=2*Pi;
            */
        unsigned int nAngleIdx = (unsigned int)std::floor( nAngle*_resStepInv + 0.5 );

        RW_DEBUG("insert: " << nAngleIdx << " angle: "<< nAngle);
        if(nAngleIdx>=_normalToContactsMap.size()){
            RW_WARN("Index out of range: " << nAngleIdx << "<" << _normalToContactsMap.size());
            nAngleIdx = _normalToContactsMap.size()-1;
        }
        _normalToContactsMap[nAngleIdx].push_back(&c);
    }
    _avgCurvature /= contourSize;
    std::cout << "- Average Object Curvature: " << _avgCurvature << std::endl;
    std::cout << "- Max Object Curvature: " << _maxCurvature << std::endl;
    std::cout << "- Min Object Curvature: " << _minCurvature << std::endl;
}

Contour2DInfoMap::ContactPtrList Contour2DInfoMap::getCNormals(double nAngle, double margin, double thres){

    //std::cout << "Angle " << nAngle << std::endl;
    int startIdx = (int)std::floor( clampAngle(nAngle-margin)*_resStepInv + 0.5 );
    int stopIdx = (int)std::floor( clampAngle(nAngle+margin)*_resStepInv + 0.5 );
    //std::cout << "Start: " << startIdx << " " << stopIdx << std::endl;
    ContactPtrList list;
    if(startIdx>stopIdx){
        for(;startIdx<_res;startIdx++){
            //std::cout << " size: " << _normalToContactsMap[startIdx].size() << std::endl;
            BOOST_FOREACH(Contact2D* cPtr, _normalToContactsMap[startIdx]){
                if(cPtr->avgCurvature<thres)
                    list.push_back(cPtr);
            }
        }
        startIdx = 0;
    }
    for(;startIdx<=stopIdx;startIdx++){
        //std::cout << " size: " << _normalToContactsMap[startIdx].size() << std::endl;
        BOOST_FOREACH(Contact2D* cPtr, _normalToContactsMap[startIdx]){
            if(cPtr->avgCurvature<thres)
                list.push_back(cPtr);
        }
    }
    return list;
}
/*
const Contact& Contour2DInfoMap::getContact(double angle){
    return _contacts[getContactIdx(angle)];
}

int Contour2DInfoMap::getContactIdx(double angle){
    if(angle>MAX_THETA) angle -= 2*Pi;
    else if(angle<MIN_THETA) angle += 2*Pi;
    return std::floor( (angle/_resStep) + 0.5 );
}
*/
