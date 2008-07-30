#include "PathLengthOptimizer.hpp"

#include <rw/pathplanning/QConstraint.hpp>
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
    /**
     * Count up the iterator with cnt
     */
    void inc(QPath::iterator& it, int cnt)
    {
        for (int i = 0; i < cnt; i++) ++it;
    }

    double calcLength(
        QPath::iterator start,
        QPath::iterator end,
        const Metric<double>& metric)
    {
        RW_ASSERT(start != end);

        QPath::iterator it1 = start;
        QPath::iterator it2 = ++start;
        double length = 0;
        for (; it1 != end; it1++, it2++) {
            length += metric.distance(*it1, *it2);
        }
        return length;
    }
}

const std::string PathLengthOptimizer::PROP_LOOPCOUNT = "LoopCount";
const std::string PathLengthOptimizer::PROP_MAXTIME = "MaxTime";
const std::string PathLengthOptimizer::PROP_SUBDIVLENGTH = "SubDivideLength";

PathLengthOptimizer::PathLengthOptimizer(const rw::pathplanning::PlannerConstraint& constraint,
                                         rw::math::MetricPtr metric) :
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
QPath PathLengthOptimizer::pathPruning(const QPath& path) {
    QPath result(path);

    QPath::iterator it1 = result.begin();
    QPath::iterator it2 = result.begin();
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
    return result;
}


QPath PathLengthOptimizer::shortCut(const QPath& path) {
    return shortCut(
        path,
        _propertyMap.get<int>(PROP_LOOPCOUNT),
        _propertyMap.get<double>(PROP_MAXTIME),
        _propertyMap.get<double>(PROP_SUBDIVLENGTH));
}

/**
 * @brief Optimizes using the shortcut technique
 *
 * The \b shortCut algorithm works by selecting two random indices i and j and
 * try to connect those.
 *
 * The algorithm will loop until either the specified \b cnt is of met or the specified
 * time is reached.
 *
 * @param path [in] Path to optimize
 * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
 * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
 * @return The optimized path
 */
QPath PathLengthOptimizer::shortCut(const QPath& path,
                                    size_t maxcnt,
                                    double time,
                                    double subDivideLength)
{
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    QPath result(path);
    resamplePath(result, subDivideLength);

    size_t cnt = 0;
    Timer timer;

    QPath::iterator it1;
    QPath::iterator it2;

    // The start and end configurations does not change
    // _localplanner->setTestQStart(false);
    // _localplanner->setTestQGoal(false);

    while (
        (maxcnt == 0 || cnt < maxcnt)  &&
        (time == 0 || timer.getTime() < time))
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

        if (calcLength(it1, it2, *_metric) <= _metric->distance(*it1, *it2))
            continue;

        if (validPath(*it1, *it2)) {
            it1 = resample(it1, it2, subDivideLength, result);
            result.erase(it1, it2);
        }
    }
    return result;
}

QPath PathLengthOptimizer::partialShortCut(const QPath& path) {
    return partialShortCut(
        path,
        _propertyMap.get<int>(PROP_LOOPCOUNT),
        _propertyMap.get<double>(PROP_MAXTIME),
        _propertyMap.get<double>(PROP_SUBDIVLENGTH));
}

/**
 * @brief Optimizes using the partial shortcut technique
 *
 * The \b partialShortCut algorithm select two random node indices i and j and a random
 * position in the configuration vector. A shortcut is then only tried between the values
 * corresponding to the random position.
 *
 * The algorithm will loop until either the specified \b cnt is of met or the specified
 * time is reached.
 *
 * @param path [in] Path to optimize
 * @param cnt [in] Max count to use. If cnt=0, only the time limit will be used
 * @param time [in] Max time to use (in seconds). If time=0, only the cnt limit will be used
 * @return The optimized path
 */
QPath PathLengthOptimizer::partialShortCut(const QPath& path,
                                           size_t maxcnt,
                                           double time,
                                           double subDivideLength)
{
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    QPath result(path);
    resamplePath(result, subDivideLength);

    Timer timer;

    size_t cnt = 0;
    QPath::iterator it1;
    QPath::iterator it2;

    // _localplanner->setTestQStart(false);
    // _localplanner->setTestQGoal(true);
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
        QPath::iterator itEnd = it2;
        it2++;

        QPath subpath;
        subpath.insert(subpath.end(),it1, it2);
        //std::list<Q> subpath(it1, it2);
        double qstart = subpath.front()(index);
        double qend = subpath.back()(index);
        double k = 0;

        RW_ASSERT(subpath.size() > 1);
        double delta = 1.0/(subpath.size()-1);

        //Make interpolator of the selected index
        for (QPath::iterator it = subpath.begin(); it != subpath.end(); it++, k += delta) {
            (*it)(index) = qstart * (1-k) + qend * k;
        }

        if (calcLength(it1, itEnd, *_metric) <= _metric->distance(*it1, *itEnd))
            continue;

        QPath::iterator itsub1 = subpath.begin();
        QPath::iterator itsub2 = itsub1;
        itsub2++;
        bool fail = false;
        for (; itsub2 != subpath.end(); itsub1++, itsub2++) {
            if (!validPath(*itsub1, *itsub2)) {
                fail = true;
                break;
            }
        }
        if (!fail) {
            QPath::iterator it = it1;
            QPath::iterator itsub = subpath.begin();
            for (; it != it2; it++, itsub++) {
                *it = *itsub;
            }
            //Skip the resampling as it does not appear to have any positive effect
            //and just makes it slower
            //resamplePath(result, subDivideLength);
        }
    }

    return result;
}

void PathLengthOptimizer::resamplePath(QPath& path, double subDivideLength)
{
    QPath::iterator it1 = path.begin();
    QPath::iterator it2 = it1;
    it2++;
    for (; it2 != path.end(); ) {
        it1 = resample(it1, it2, subDivideLength, path);
        it2 = it1;
        it2++;
    }
}

QPath::iterator PathLengthOptimizer::resample(QPath::iterator it1,
                                              QPath::iterator it2,
                                              double subDivideLength,
                                              QPath& result)
{
    if (subDivideLength == 0)
        return ++it1;

    const Q& q1 = *it1;
    const Q& q2 = *it2;
    double length = _metric->distance(q1, q2);

    int stepcount = (int)std::ceil(length / subDivideLength);

    // Avoid division by zero.
    if (stepcount > 1) {
        double delta = 1.0 / (double)stepcount;

        for (int i = 1; i < stepcount; i++) {
            Q qnew = (1 - delta * i) * q1 + (delta * i) * q2;
            it1 = result.insert(++it1, qnew);
        }
    }

    return ++it1;
}

bool PathLengthOptimizer::validPath(const rw::math::Q& from,
                                    const rw::math::Q& to)
{
    return !_constraint.getQConstraint().inCollision(to) &&
           !_constraint.getQEdgeConstraint().inCollision(from, to);
}
