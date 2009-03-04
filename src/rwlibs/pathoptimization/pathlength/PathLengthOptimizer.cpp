/*********************************************************************
 * RobWork Version 0.3
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

#include "PathLengthOptimizer.hpp"

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/PlannerUtil.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Metric.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rw::trajectory;
using namespace rwlibs::pathoptimization;

namespace
{
    typedef PathLengthOptimizer::QList QList;

    /**
       Count up the iterator with cnt
    */
    void inc(QList::iterator& it, int cnt)
    {
        for (int i = 0; i < cnt; i++) ++it;
    }

    double pathLength(
        QList::iterator start,
        QList::iterator end,
        const QMetric& metric)
    {
        RW_ASSERT(start != end);

        // Math::pathLength() does not include the end iterator in the sequence.
        return Math::pathLength(start, ++end, metric);
    }
}

const std::string PathLengthOptimizer::PROP_LOOPCOUNT = "LoopCount";
const std::string PathLengthOptimizer::PROP_MAXTIME = "MaxTime";
const std::string PathLengthOptimizer::PROP_SUBDIVLENGTH = "SubDivideLength";

PathLengthOptimizer::PathLengthOptimizer(
    const PlannerConstraint& constraint,
    QMetricPtr metric)
    :
    _constraint(constraint),
    _metric(metric)
{
    _propertyMap.add(PROP_LOOPCOUNT, "Maximal Number of Loops", 1000);
    _propertyMap.add(PROP_MAXTIME, "Maximal Time to use (seconds)", 200.0);
    _propertyMap.add(PROP_SUBDIVLENGTH, "Subdivide Length", 0.1);
}

PathLengthOptimizer::~PathLengthOptimizer() {}

PropertyMap& PathLengthOptimizer::getPropertyMap()
{
    return _propertyMap;
}

/**
 * Runs through the path an tests if nodes with
 * index i and i+2 can be directly connected. If so it removed node i+1.
 */
void PathLengthOptimizer::pathPruning(QList& result)
{
    QList::iterator it1 = result.begin();
    QList::iterator it2 = result.begin();
    it2++; it2++;

    while (it2 != result.end()) {
        if (validPath(*it1, *it2)) {
            it2 = result.erase(++it1);
            it1 = it2;
            it1--;
            it2++;
        } else {
            it1++;
            it2++;
        }
    }
}

void PathLengthOptimizer::shortCut(QList& path)
{
    shortCut(
        path,
        _propertyMap.get<int>(PROP_LOOPCOUNT),
        _propertyMap.get<double>(PROP_MAXTIME),
        _propertyMap.get<double>(PROP_SUBDIVLENGTH));
}

void PathLengthOptimizer::shortCut(QList& result,
                                   size_t maxcnt,
                                   double time,
                                   double subDivideLength)
{
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    resamplePath(result, subDivideLength);

    size_t cnt = 0;
    Timer timer;

    QList::iterator it1;
    QList::iterator it2;

    // The start and end configurations does not change
    setTestQStart(false);
    setTestQEnd(false);

    while ((maxcnt == 0 || cnt < maxcnt) && (time == 0 || timer.getTime() < time))
    {
        cnt++;
        const size_t n = result.size();

        // We need this or else things below crash.
        if (n == 2) break;

        it1 = result.begin();
        it2 = result.begin();
        const int i1 = Math::ranI(0, n - 2);
        const int i2 = Math::ranI(i1 + 2, n);

        inc(it1, i1);
        inc(it2, i2);

		if (pathLength(it1, it2, *_metric) <= _metric->distance(*it1, *it2))
            continue;

        if (validPath(*it1, *it2)) {
            it1 = resample(it1, *it2, subDivideLength, result);
            result.erase(it1, it2);
        }
    }
}

void PathLengthOptimizer::partialShortCut(QList& path)
{
    partialShortCut(
        path,
        _propertyMap.get<int>(PROP_LOOPCOUNT),
        _propertyMap.get<double>(PROP_MAXTIME),
        _propertyMap.get<double>(PROP_SUBDIVLENGTH));
}

void PathLengthOptimizer::partialShortCut(
    QList& result,
    size_t maxcnt,
    double time,
    double subDivideLength)
{
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    resamplePath(result, subDivideLength);

    Timer timer;

    size_t cnt = 0;
    QList::iterator it1;
    QList::iterator it2;

    setTestQStart(false);
    setTestQEnd(true);

    while (
        (maxcnt == 0 || cnt < maxcnt) &&
        (time == 0 || timer.getTime() < time))
    {
        cnt++;
        size_t n = result.size();
        it1 = result.begin();
        it2 = result.begin();

        int i1 = Math::ranI(0, n-2);
        int i2 = Math::ranI(i1+2, n);
        int index = Math::ranI(0, result.front().size());

        inc(it1, i1);
        inc(it2, i2);
        QList::iterator itEnd = it2;
        it2++;

        QList subpath;
        subpath.insert(subpath.end(),it1, it2);
        double qstart = subpath.front()(index);
        double qend = subpath.back()(index);
        double k = 0;

        RW_ASSERT(subpath.size() > 1);
        double delta = 1.0/(subpath.size()-1);

        //Make interpolator of the selected index
        for (QList::iterator it = subpath.begin(); it != subpath.end(); it++, k += delta) {
            (*it)(index) = qstart * (1-k) + qend * k;
        }

        if (pathLength(it1, itEnd, *_metric) <= _metric->distance(*it1, *itEnd))
            continue;

        QList::iterator itsub1 = subpath.begin();
        QList::iterator itsub2 = itsub1;
        itsub2++;
        bool fail = false;
        for (; itsub2 != subpath.end(); itsub1++, itsub2++) {
            if (!validPath(*itsub1, *itsub2)) {
                fail = true;
                break;
            }
        }
        if (!fail) {
            QList::iterator it = it1;
            QList::iterator itsub = subpath.begin();
            for (; it != it2; it++, itsub++) {
                *it = *itsub;
            }
            //Skip the resampling as it does not appear to have any positive effect
            //and just makes it slower
            //resamplePath(result, subDivideLength);
        }
    }
}

void PathLengthOptimizer::resamplePath(QList& path, double subDivideLength)
{
    QList::iterator it1 = path.begin();
    QList::iterator it2 = it1; ++it2;
    for (; it2 != path.end(); ) {
        it1 = resample(it1, *it2, subDivideLength, path);
        it2 = it1; ++it2;
    }
}

QList::iterator PathLengthOptimizer::resample(QList::iterator it1,
                                              const Q& q2,
                                              double subDivideLength,
                                              QList& result)
{
    if (subDivideLength == 0) return ++it1;

    const Q& q1 = *it1;
    const double length = _metric->distance(q1, q2);

    const int stepcount = (int)std::ceil(length / subDivideLength);

    // Avoid division by zero.
    if (stepcount > 1) {
        const double delta = 1.0 / (double)stepcount;

        for (int i = 1; i < stepcount; i++) {
            Q qnew = (1 - delta * i) * q1 + (delta * i) * q2;
            it1 = result.insert(++it1, qnew);
        }
    }

    return ++it1;
}

bool PathLengthOptimizer::validPath(
    const Q& from,
    const Q& to)
{
    return !PlannerUtil::inCollision(
        _constraint,
        from,
        to,
        _testQStart,
        _testQEnd);
}

//----------------------------------------------------------------------

QPath PathLengthOptimizer::pathPruning(const QPath& path)
{
    QList tmp(path.begin(), path.end());
    pathPruning(tmp);
    return QPath(tmp.begin(), tmp.end());
}

QPath PathLengthOptimizer::shortCut(const QPath& path,
                                    size_t cnt,
                                    double time,
                                    double subDivideLength)
{
    if (path.empty()) return path;

    QList tmp(path.begin(), path.end());
    shortCut(tmp, cnt, time, subDivideLength);
    return QPath(tmp.begin(), tmp.end());
}

QPath PathLengthOptimizer::shortCut(const QPath& path)
{
    QList tmp(path.begin(), path.end());
    shortCut(tmp);
    return QPath(tmp.begin(), tmp.end());
}

QPath PathLengthOptimizer::partialShortCut(const QPath& path,
                                           size_t cnt,
                                           double time,
                                           double subDivideLength)
{
    QList tmp(path.begin(), path.end());
    partialShortCut(tmp, cnt, time, subDivideLength);
    return QPath(tmp.begin(), tmp.end());
}

QPath PathLengthOptimizer::partialShortCut(const QPath& path)
{
    QList tmp(path.begin(), path.end());
    partialShortCut(tmp);
    return QPath(tmp.begin(), tmp.end());
}
