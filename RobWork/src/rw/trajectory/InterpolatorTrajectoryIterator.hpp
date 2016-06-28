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


#ifndef RW_TRAJECTORY_INTERPOLATORTRAJECTORYITERATOR_HPP
#define RW_TRAJECTORY_INTERPOLATORTRAJECTORYITERATOR_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include "TrajectoryIterator.hpp"

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Bi-directional iterator for running efficiently through a trajectory
     */
    template <class T>
	class InterpolatorTrajectoryIterator: public TrajectoryIterator<T>
    {
    public:
        /**
         * @brief Constructs iterator for \b trajectory
         *
         * @param trajectory [in] Trajectory to iterate through
         * @param dt [in] Default stepsize used for ++ and -- operators
         */
        InterpolatorTrajectoryIterator(const InterpolatorTrajectory<T>* trajectory, double dt = 1)
        {
            _trajectory = trajectory;
            _dt = dt;
            _time = 0;
            _currentSegment = trajectory->_segments.begin();
        }

        /**
         * @copydoc TrajectoryIterator::getTime()
         */
        double getTime() { return _time; }

        /**
         * @copydoc TrajectoryIterator::operator-=()
         */
        void operator -=(double dt)
        {
            if (_time - dt < 0)
                _time = 0;
            else
                _time -= dt;
            while (_time < _currentSegment->t1 )
                _currentSegment--;
        }

        /**
         * @copydoc TrajectoryIterator::operator +=()
         */
        void operator +=(double dt)
        {
            if (_time + dt > _trajectory->duration())
                _time = _trajectory->duration();
            else
                _time += dt;
            while (_time > _currentSegment->t2 )
                _currentSegment++;
        }

        /**
         * @copydoc TrajectoryIterator::operator++()
         */

        TrajectoryIterator& operator++()
        {
            (*this) += _dt;
            return *this;
        }

        /**
         * @copydoc TrajectoryIterator::operator--()
         */
        TrajectoryIterator& operator--()
        {
            (*this) -= _dt;
            return *this;
        }

        /**
         * @copydoc TrajectoryIterator::isEnd()
         */
        bool isEnd() { return _time >= _trajectory->duration(); }

        /**
         * @copydoc TrajectoryIterator::isBegin()
         */
        bool isBegin() { return _time <= 0; }

        /**
         * @copydoc TrajectoryIterator::operator*()
         */
        T operator*() const { return x(); }

        /**
         * @copydoc TrajectoryIterator::x()
         */
        T x() const {
            return _trajectory->getX(*_currentSegment, _time);
        }

        /**
         * @copydoc TrajectoryIterator::dx()
         */
        T dx() const {
            return _trajectory->getDX(*_currentSegment, _time);
        }

        /**
         * @copydoc TrajectoryIterator::ddx()
         */
        T ddx() const {
            return _trajectory->getDDX(*_currentSegment, _time);
        }

    private:
        typename Trajectory<T>::SegmentList::const_iterator _currentSegment;
        const Trajectory<T>* _trajectory;
        double _time;
        double _dt;
    };

    /** @} */

}} // end namespaces

#endif // end include guard
