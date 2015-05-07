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


#ifndef RW_TRAJETORY_TRAJETORYSEQUENCE
#define RW_TRAJETORY_TRAJETORYSEQUENCE

#include <rw/trajectory/Trajectory.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Q.hpp>
#include <boost/foreach.hpp>

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/

/**
 * @brief Combines a number of trajectories.
 *
 * Takes an arbitrary number of trajectories and combines them. The start time of the first 
 * trajectory determines the start time of the union. The remaining trajectories are appended
 * discardless of their start time. The duration of the union corresponds to the sum of the 
 * duration of all the trajectories.
 *
 * The value returned in the transition between two trajectories, corresponds to the end point of
 * the leading trajectory. 
 *
 * Access to a value is O(lg n) with n being the number of trajectories combined.
 */
template <class T>
class TrajectorySequence: public Trajectory<T> 
{
public:
	//! @brief smart pointer type
     typedef rw::common::Ptr<TrajectorySequence<T> > Ptr;

	/**
	 * @brief Create a trajectory from \b trajectories.
	 * 
	 * @param trajectories [in] Trajectories to join. 
	 */
	TrajectorySequence(const std::vector<typename Trajectory<T>::Ptr> trajectories):
	  _trajectories(trajectories)
	{
		initialize();  
	}

	/**
	 * @brief Create a trajectory from \b trajectory1 and \ trajectory2.
	 * 
	 * @param trajectory1 [in] First trajectory
	 * @param trajectory2 [in] Second trajectory
	 */
	TrajectorySequence(typename Trajectory<T>::Ptr trajectory1, typename Trajectory<T>::Ptr trajectory2) {
		_trajectories.push_back(trajectory1);
		_trajectories.push_back(trajectory2);
		
		initialize();
	}


    /**
	 * @copydoc Trajectory::x
     */
	virtual T x(double t) const {
		int idx = index(t);
		t = time(t, idx);
		
		return _trajectories[idx]->x(t);
	}

    /**
	 * @copydoc Trajectory::dx
     */
	virtual T dx(double t) const {
		int idx = index(t);
		t = time(t, idx);
		
		return _trajectories[idx]->dx(t);
	}

    /**
	 * @copydoc Trajectory::ddx
     */
	virtual T ddx(double t) const {
		int idx = index(t);
		t = time(t, idx);
		
		return _trajectories[idx]->ddx(t);
	}

    /**
	 * @copydoc Trajectory::duration
     */
	virtual double duration() const {
		if (_times.size() > 0)
			return _times.back() - startTime();
		else
			return 0;
	}


    /**
	 * @copydoc Trajectory::startTime
     */
	virtual double startTime() const {
		return _trajectories.front()->startTime();
	}



private:
	std::vector<typename Trajectory<T>::Ptr> _trajectories;

	std::vector<double> _times;
	
	void initialize() {
		double last = 0;
		if (_trajectories.size() > 0)
			last = _trajectories.front()->startTime();

		BOOST_FOREACH(typename Trajectory<T>::Ptr& traj, _trajectories) {
			last += traj->duration();
			_times.push_back(last);
		}
	}

	int index(double t) const {
		std::vector<double>::const_iterator it = std::lower_bound(_times.begin(), _times.end(), t);
		size_t res = size_t(it - _times.begin());
		if (res >= _trajectories.size())
			RW_THROW("Index of trajectory is out of bounds");
		return (int)res;
			
	}

	double time(double t, size_t index) const {
		if (index == 0)
			return t;
		else 
			return t-_times[index-1];
	}

		/**
		 * @brief Bi-directional iterator for running efficiently through a trajectory
		 */
		template <class U>
		class TrajectorySequenceIterator: public TrajectoryIterator<U>
		{
		public:
			/**
			 * @brief Constructs iterator for \b trajectory
			 *
			 * @param trajectory [in] Trajectory to iterate through
			 * @param dt [in] Default stepsize used for ++ and -- operators
			 */
			TrajectorySequenceIterator(typename TrajectorySequence<U>::Ptr trajectory, double dt = 1):			  
			  _trajectories(trajectory->_trajectories)
			{
				_trajectory = trajectory;
				
				_dt = dt;
				_time = _trajectory->startTime();
				_currentIndex = 0;
			}

			/**
			 * @copydoc TrajectoryIterator::getTime()
			 */
			double getTime() const { return _time; }

			/**
			 * @copydoc TrajectoryIterator::dec(double)
			 */
			virtual void dec(double dt)
			{				
				if (_time - dt < _trajectory->startTime()) {
					_time = _trajectory->startTime();
					_currentIndex = 0;
				}
				else {
					_time -= dt;
					while (_currentIndex > 0 && _trajectory->_times[_currentIndex-1] >= _time)
						_currentIndex--;
				}
			}

			/**
			 * @copydoc TrajectoryIterator::inc(double)
			 */
			virtual void inc(double dt)
			{
				if (_time + dt > _trajectory->duration()) {
					_time = _trajectory->duration();
					_currentIndex = _trajectory->_trajectories.size()-1;
				}
				else {
					_time += dt;
					while (_trajectory->_times[_currentIndex]<_time)
						_currentIndex++;					
				}
			}

			/**
			 * @copydoc TrajectoryIterator::inc()
			 */
			virtual void inc()
			{
				inc(_dt);
			}

			/**
			 * @copydoc TrajectoryIterator::operator--()
			 */
			virtual void dec()
			{
				dec(_dt);
			}

			/**
			 * @copydoc TrajectoryIterator::isEnd()
			 */
			bool isEnd() const { return _time >= _trajectory->endTime(); }

			/**
			 * @copydoc TrajectoryIterator::isBegin()
			 */
			bool isBegin() const { return _time <= _trajectory->startTime(); }

			/**
			 * @copydoc TrajectoryIterator::operator*()
			 */
			T operator*() const { return x(); }

			/**
			 * @copydoc TrajectoryIterator::x()
			 */
			T x() const {
				const Trajectory<rw::math::Q>::Ptr& traj = _trajectory->_trajectories[_currentIndex];
				return traj->x(_trajectory->time(_time, _currentIndex));
			}

			/**
			 * @copydoc TrajectoryIterator::dx()
			 */
			T dx() const {
				const Trajectory<rw::math::Q>::Ptr& traj = _trajectory->_trajectories[_currentIndex];
				return traj->dx(_trajectory->time(_time, _currentIndex));
			}

			/**
			 * @copydoc TrajectoryIterator::ddx()
			 */
			T ddx() const {
				const Trajectory<rw::math::Q>::Ptr& traj = _trajectory->_trajectories[_currentIndex];
				return traj->ddx(_trajectory->time(_time, _currentIndex));
			}

		private:
			
			typename TrajectorySequence<U>::Ptr _trajectory;
			std::vector<typename Trajectory<U>::Ptr >& _trajectories;
			size_t _currentIndex;
			double _time;
			double _dt;
		};




public:
	
	typename TrajectoryIterator<T>::Ptr getIterator(double dt) const {
		return rw::common::ownedPtr(new TrajectorySequenceIterator<T>(const_cast<TrajectorySequence<T>*>(this), dt));
	}


};

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif // end namespaces
