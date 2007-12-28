#include "PathLengthOptimizer.hpp"

using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rwlibs::pathoptimization;

PathLengthOptimizer::PathLengthOptimizer(Device* device,
                                         const State& state,
                                         boost::shared_ptr<CollisionDetector> collisionDetector,
                                         double resolution)
{
    _localplanner = new StraightLinePathPlanner(device, state, collisionDetector.get(), resolution)
}

PathLengthOptimizer::PathLengthOptimizer(PathPlanner* pathplanner):
    _localplanner(localplanner)
{
}

	
PathLengthOptimizer::~PathLengthOptimizer() {
    delete _localplanner;
}
	


bool PathLengthOptimizer::testSegment(const Q& q1, const Q& q2) {
    
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
Path PathLengthOptimizer::shortCut(const Path& path, size_t maxcnt, double time) {
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    Path result(path);
    size_t cnt = 0;
    Timer timer;
    timer.reset();
    Path::iterator it1;
    Path::iterator it2;
    
    size_t n = path.size();
    while ( (maxcnt == 0 || cnt < maxcnt) && (time == 0 || time < timer.getTime())) {
        it1 = result.begin();
        it2 = result.begin();
        int i1 = Math::RanI(0, n-2);
        int i2 = Math::RanI(i1+2, n);
        it1 += i1; //If this does not compile use a for loop
        it2 += i2; //If this does not compile use a for loop
        
        Path subpath;
        if (_localplanner->query(*it1, *it2, subpath, 0)) {
            it1++;
            result.erase(it1, it2);           
        }
        cnt++;
    }
    return result;
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
Path PathLengthOptimizer::partialShortCut(const Path& path, size_t cnt, double time) {
    if (maxcnt == 0 && time == 0)
        RW_THROW("With maxcnt == 0 and time == 0 the algorithm will never terminate");

    Path result(path);
    size_t cnt = 0;
    Timer timer;
    timer.reset();
    Path::iterator it1;
    Path::iterator it2;
    
    size_t n = path.size();
    
    while ( (maxcnt == 0 || cnt < maxcnt) && (time == 0 || time < timer.getTime())) {
        it1 = result.begin();
        it2 = result.begin();
        int i1 = Math::RanI(0, n-2);
        int i2 = Math::RanI(i1+2, n);
        int index = Math::Ran(0, result.front().size());
        it1 += i1;
        it2 += i2;
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
        
        Path::iterator itsub1 = subpath.begin();
        Path::iterator itsub2 = itsub1++;
        bool fail = false;
        for (; itsub2 != subpath.end(); itsub1++, itsub2++) {
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
        }                
    }
}



