#include "PathLengthOptimizer.hpp"

#include <rw/pathplanning/StraightLinePathPlanner.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::pathplanning;
using namespace rwlibs::pathoptimization;

namespace {
    /**
     * Count up the iterator with cnt
     */
    void inc(Path::iterator& it, int cnt) {
        for (int i = 0; i<cnt; i++)
            it++;
    }

    double calcLength(Path::iterator start, Path::iterator end, Metric<double>* metric) {
        Path::iterator it1 = start;
        Path::iterator it2 = ++start;
        double length = 0;
        for (; it1 != end; it1++, it2++) {
            length += metric->distance(*it1, *it2);
        }
        return length;
    }
}


const std::string PathLengthOptimizer::PROP_LOOPCOUNT = "LoopCount";
const std::string PathLengthOptimizer::PROP_MAXTIME = "MaxTime";
const std::string PathLengthOptimizer::PROP_SUBDIVLENGTH = "SubDivideLength";


PathLengthOptimizer::PathLengthOptimizer(
    Device* device,
    const State& state,
    boost::shared_ptr<CollisionDetector> collisionDetector,
    double resolution,
    boost::shared_ptr<Metric<double> > metric)
    :
    _metric(metric)
{
    _localplanner =
        new StraightLinePathPlanner(device, state, collisionDetector.get(), resolution);
    initialize();
}

PathLengthOptimizer::PathLengthOptimizer(
    PathPlanner* localplanner,
    boost::shared_ptr<Metric<double> > metric)
    :
    _metric(metric),
    _localplanner(localplanner)
{
    initialize();
}

void PathLengthOptimizer::initialize()
{
    _propertyMap.add(PROP_LOOPCOUNT, "Maximal Number of Loops", 1000);
    _propertyMap.add(PROP_MAXTIME, "Maximal Time to use (seconds)", 200.0);
    _propertyMap.add(PROP_SUBDIVLENGTH, "Subdivide Length", 0.1);
}

PathLengthOptimizer::~PathLengthOptimizer() {
    delete _localplanner;
}

PropertyMap& PathLengthOptimizer::getPropertyMap() {
    return _propertyMap;
}



/**
 * Runs through the path an tests if nodes with
 * index i and i+2 can be directly connected. If so it removed node i+1.
 */
Path PathLengthOptimizer::pathPruning(const Path& path) {
    Path result(path);

    Path::iterator it1 = result.begin();
    Path::iterator it2 = result.begin();
    it2++; it2++;

    while (it2 != result.end()) {
        Path subpath;
        if (_localplanner->query(*it1, *it2, subpath, 0)) {
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


Path PathLengthOptimizer::shortCut(const Path& path) {
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
Path PathLengthOptimizer::shortCut(const Path& path, size_t maxcnt, double time, double subDivideLength) {
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    Path result(path);
    resamplePath(result, subDivideLength);

    size_t cnt = 0;
    Timer timer;
    timer.reset();
    Path::iterator it1;
    Path::iterator it2;


    //The start and end configurations does not change
    _localplanner->setTestQStart(false);
    _localplanner->setTestQGoal(false);
    while ( (maxcnt == 0 || cnt < maxcnt)  && (time == 0 || timer.getTime() < time) ) {
        cnt++;
        size_t n = result.size();
        it1 = result.begin();
        it2 = result.begin();
        int i1 = Math::RanI(0, n-2);
        int i2 = Math::RanI(i1+2, n);
        //std::cout<<"i1 = "<<i1<<"  i2 = "<<i2<<std::endl;
        inc(it1, i1);
        inc(it2, i2);

        if (calcLength(it1, it2, _metric.get()) <= _metric->distance(*it1, *it2))
            continue;

        Path subpath;
        if (_localplanner->query(*it1, *it2, subpath, 0)) {
            it1 = resample(it1, it2, subDivideLength, result);
            result.erase(it1, it2);
        }
    }
    return result;
}

Path PathLengthOptimizer::partialShortCut(const Path& path) {
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
Path PathLengthOptimizer::partialShortCut(const Path& path, size_t maxcnt, double time, double subDivideLength) {
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    Path result(path);
    resamplePath(result, subDivideLength);

    size_t cnt = 0;
    Timer timer;
    timer.reset();
    Path::iterator it1;
    Path::iterator it2;

    _localplanner->setTestQStart(false);
    _localplanner->setTestQGoal(true);
    while ( (maxcnt == 0 || cnt < maxcnt) && (time == 0 || timer.getTime() < time)) {
        cnt++;
        size_t n = result.size();
        it1 = result.begin();
        it2 = result.begin();
        int i1 = Math::RanI(0, n-2);
        int i2 = Math::RanI(i1+2, n);
        int index = Math::RanI(0, result.front().size());
        inc(it1, i1);
        inc(it2, i2);
        Path::iterator itEnd = it2;
        it2++;

        Path subpath(it1, it2);
        double qstart = subpath.front()(index);
        double qend = subpath.back()(index);
        double k = 0;
        double delta = 1.0/(subpath.size()-1);
        //Make interpolator of the selected index
        for (Path::iterator it = subpath.begin(); it != subpath.end(); it++, k += delta) {
            (*it)(index) = qstart * (1-k) + qend * k;
        }

        if (calcLength(it1, itEnd, _metric.get()) <= _metric->distance(*it1, *itEnd))
            continue;

        Path::iterator itsub1 = subpath.begin();
        Path::iterator itsub2 = itsub1;
        itsub2++;
        bool fail = false;
        for (; itsub2 != subpath.end(); itsub1++, itsub2++) {
            Path tmppath;
            if (!_localplanner->query(*itsub1, *itsub2, tmppath, time - timer.getTime())) {
                fail = true;
                break;
            }
        }
        if (!fail) {
            Path::iterator it = it1;
            Path::iterator itsub = subpath.begin();
            for (; it != it2; it++, itsub++) {
                *it = *itsub;
            }
            //Skip the resampling as it does not appear to have any positive effect
            //and just makes it slower
            //resamplePath(result, subDivisionSize);
        }

    }
    return result;
}


void PathLengthOptimizer::resamplePath(Path& path, double subDivisionSize) {
    Path::iterator it1 = path.begin();
    Path::iterator it2 = it1;
    it2++;
    for (; it2 != path.end(); ) {
        it1 = resample(it1, it2, subDivisionSize, path);
        it2 = it1;
        it2++;
    }
}

Path::iterator PathLengthOptimizer::resample(Path::iterator it1, Path::iterator it2, double subDivisionSize, Path& result) {
    if (subDivisionSize == 0)
        return ++it1;

    const Q& q1 = *it1;
    const Q& q2 = *it2;
    double length = _metric->distance(q1, q2);
    int stepcount = (int)(std::ceil(length/subDivisionSize));
    double delta = 1.0/(double)stepcount;
    for (int i = 1; i<stepcount; i++) {
        Q qnew = (1-delta*i)*q1 + (delta*i)*q2;
        it1 = result.insert(++it1, qnew);
    }
    return ++it1;
}
