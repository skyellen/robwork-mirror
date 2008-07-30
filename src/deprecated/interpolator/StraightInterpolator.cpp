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

#include "StraightInterpolator.hpp"

#include <assert.h>

using namespace rw::interpolator;
using namespace rw::math;



StraightInterpolator::StraightInterpolator(const rw::math::Q& qStart,
                     const std::vector<std::pair<rw::math::Q, double> >& dataset) {
    _viaPoints.push_back(qStart);
    _times.push_back(0);
    typedef std::vector<std::pair<Q, double> >::const_iterator I;
    for (I it = dataset.begin(); it != dataset.end(); ++it) {
        _segments.push_back(new StraightSegment(_viaPoints.back(), (*it).first-_viaPoints.back(), (*it).second-_times.back()));
        _viaPoints.push_back((*it).first);
        _times.push_back((*it).second);
    }
    _length = _times.back();
}

StraightInterpolator::StraightInterpolator(const rw::math::Q &qStart,                                          
                                           const rw::math::Q &qEnd,
                                           double tEnd):
                                               _length(0)
{
    _viaPoints.push_back(qStart);
    _times.push_back(0);
    addVia(qEnd, tEnd);
}


StraightInterpolator::~StraightInterpolator() {
    for (std::vector<FunctionSegment*>::iterator it = _segments.begin(); it != _segments.end(); ++it) 
        delete (*it);
}
    
/*StraightInterpolator::StraightInterpolator(const rw::math::Q &qStart,
                                           const rw::math::Q &qEnd,
                                           const Metric<>& metric):
    _length(0)
{
    _viaPoints.push_back(qStart);
    addVia(qEnd);
}*/

void StraightInterpolator::addVia(const rw::math::Q &via, double time)
{
    _segments.push_back(new StraightSegment(_viaPoints.back(), via-_viaPoints.back(), time-_times.back()));
    _viaPoints.push_back(via);
    _times.push_back(time);

    _length = _times.back()-_times.front();
}

bool StraightInterpolator::insertVia(const rw::math::Q &via, double time)
{
    if (time>_times.back()) {
        addVia(via, time);
        return true;
    }
    
    std::vector<FunctionSegment*>::iterator segit = _segments.begin();
    std::vector<Q>::iterator qit = _viaPoints.begin();
    for (std::vector<double>::iterator it = _times.begin(); it != _times.end(); ++it, ++segit, ++qit) {
        if (*it>time) {
            Q qtmp = *qit;            
            double nexttime = *it;
            double pretime = *(it-1);
            
            //Insert the new via point and the time
            qit = _viaPoints.insert(qit, via);
            it = _times.insert(it, time);
            
            //Construct and insert the new segment
            Q b = (qtmp -(*qit));
            StraightSegment* seg = new StraightSegment(via,b, nexttime-time);            
            segit = _segments.insert(segit, seg);
            //Update previous segments
            if (segit != _segments.begin()) {
                b = ((*qit)-*(qit-1));
                ((StraightSegment*)*(segit-1))->setParameters(*(qit-1), b, time-pretime);
            }
            _length = _times.back()-_times.front();
            return true;
            
        }
    }
   return false;
/*    if (i >= _viaPoints.size())
        return false;

    // insert viarw::math::Q
    _viaPoints.insert(_viaPoints.begin()+i, via);

    // update the neighboring segments
    rw::math::Q b = (_viaPoints[i+1]-_viaPoints[i]);
    StraightSegment *seg = new StraightSegment(via,b);
    _segments.insert(_segments.begin()+i, seg);
    _length += 1.0;

    // update segment i-1 if it exist
    if (i > 0) { // only update presegment if its not the first via
        _length -= _segments[i-1]->getIntervalLength();
        _length += 1.0;
        b = (_viaPoints[i]-_viaPoints[i-1]);
        StraightSegment *myseg = (StraightSegment*)(_segments[i-1]);
        myseg->setParameters(
            _viaPoints[i-1], b, std::make_pair(0.0, 1.0));
    }

    return true;*/
}

bool StraightInterpolator::removeVia(unsigned int i)
{
    if (i >= _viaPoints.size())
        return false;

    _viaPoints.erase(_viaPoints.begin() + i);

    delete _segments[i];

    _segments.erase( _segments.begin() + i);
    
    _times.erase( _times.begin() + i);

    // update segment i-1 if it exist
    if (i > 0){
        rw::math::Q b = (_viaPoints[i]-_viaPoints[i-1]);
        StraightSegment *myseg = (StraightSegment*)(_segments[i-1]);
        myseg->setParameters(_viaPoints[i-1],b, _times[i]-_times[i-1]);
    }
    _length = _times.back()-_times.front();
    return true;
}


Q StraightInterpolator::getX(double d) const {
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

