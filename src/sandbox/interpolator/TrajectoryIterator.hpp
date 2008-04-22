#ifndef RW_SANDBOX_TRAJECTORYITERATOR_HPP
#define RW_SANDBOX_TRAJECTORYITERATOR_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include "Trajectory.hpp"

#include <vector>

namespace rw {
namespace sandbox {

/** @addtogroup interpolator */
/*@{*/

    
/**
 * @brief Bi-directional iterator for running efficiently through a trajectory
 */    
template <class T>
class TrajectoryIterator
{   
public:
    /**
     * @brief Constructs iterator for \b trajectory
     * 
     * @param trajectory [in] Trajectory to iterate through  
     */
	TrajectoryIterator(const Trajectory<T>* trajectory) {
	    _trajectory = trajectory;
	    _time = 0;
	    _currentSegment = trajectory->_segments.begin();	    
	}
	
	/**
	 * @brief Destructor
	 */
	~TrajectoryIterator() {
	    
	}
	
	/**
	 * @brief Returns the current position (time) of the iterator
	 * @return The current time.
	 */
	double getTime() {
	    return _time;
	}
	
    /**
     * @brief This function can be used to decrease the iterator position.
     * The position can be decreased no longer than to time equals 0.
     *
     * @param val [in] a double that describes how much to decrease the
     * iterator position
     */
    void operator -=(double val) {
        if (_time - val < 0)
            _time = 0;
        else
            _time -= val;
        while (_time < _currentSegment->t1 )
            _currentSegment--;
    }

    /**
     * @brief This function can be used to increase the iterator position.
     * The position can be increased no longer than the length of the
     * complete trajectory.
     *
     * @param val [in] a double that describes how much to increase the
     * iterator position
     */
    void operator +=(double val) {
        if (_time + val > _trajectory->getLength())
            _time = _trajectory->getLength();
        else
            _time += val;
        while (_time > _currentSegment->t2 )
            _currentSegment++;

    }

    /**
     * @brief Test if the end of the trajectory is reached.
     *
     * @return true if the iterator has reached the end of the trajectory false
     * otherwise.
     */
    bool isEnd() { 
        return _time >= _trajectory->getLength();
    }
    /**
     * @brief Test if the beginning of the trajectory is reached.
     *
     * @return true if the iterator has reached the beginning of the trajectory
     * false otherwise.
     */
    bool isBegin() { 
        return _time <= 0;
    }

    /**
     * @brief Extracts a point at the current position in the trajectory.
     *
     * @param iter [in] the trajectory iterator
     *
     * @return the point at the current position in the trajectory.
     */
    friend T operator*(TrajectoryIterator& iter){
        return iter.getX();
    }

    /**
     * @brief Extracts a point at the current position in the trajectory.
     *
     * @return the point at the current position in the trajectory.
     */
    T x() const {
        return _trajectory->getX(*_currentSegment, _time);
    }

    /**
     * @brief Extracts a point of the derivative of the trajectory
     * at the current position in the trajectory
     *
     * @return the derived point at the current position in the trajectory.
     */
    T dx() const {
        return _trajectory->getDX(*_currentSegment, _time);
    }

    /**
     * @brief Extracts a point of the double derivative of the trajectory at the
     * current position in the trajectory
     *
     * @return the double derived point at the current position in the trajectory.
     */
    T ddx() const {
        return _trajectory->getDDX(*_currentSegment, _time);
    }
    
private:
    typename Trajectory<T>::SegmentList::iterator _currentSegment;
    Trajectory<T>* _trajectory;
    double _time;

    
};

/** @} */

} //end namespace sandbox
} //end namespace rw 
#endif //RW_SANDBOX_TRAJECTORYITERATOR_HPP
