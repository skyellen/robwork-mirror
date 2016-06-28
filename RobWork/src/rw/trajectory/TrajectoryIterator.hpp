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


#ifndef RW_TRAJECTORY_TRAJECTORYITERATOR_HPP
#define RW_TRAJECTORY_TRAJECTORYITERATOR_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include <rw/common/Ptr.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Bi-directional iterator for running efficiently through a trajectory
     */
	template <class T>
    class TrajectoryIterator
    {
    public:

		//! @brief smart pointer type
        typedef rw::common::Ptr<TrajectoryIterator<T> > Ptr;

        /**
         * @brief Returns the current position (time) of the iterator
         * @return The current time.
         */
        virtual double getTime() const = 0;


		/**
         * @brief Method for increasing the position of the iterator a fixed amount
		 *
         * The increment is equal to the \b dt specified in the constructor.
         */
		virtual void inc() = 0;

		/**
         * @brief Method for increasing the position of the iterator by \b dt
		 *
		 * @param dt [in] Amount to increase. A positive value is expected. 		 
         */
		virtual void inc(double dt) = 0;

		/**
         * @brief Method for decreasing the position of the iterator a fixed amount
		 *
         * The decrement is equal to the \b dt specified in the constructor.
         */
		virtual void dec() = 0;

		/**
         * @brief Method for decreasing the position of the iterator a fixed amount
		 *
		 * @param dt [in] Amount to decrease. A positive value is expected		 
         */
		virtual void dec(double dt) = 0;



        /**
         * @brief This function can be used to decrease the iterator position.
         * The position can be decreased no longer than to time equals 0.
         *
         * @param dt [in] a double that describes how much to decrease the
         * iterator position
         */
		virtual void operator -=(double dt) {
			dec(dt);
		}
        /**
         * @brief This function can be used to increase the iterator position.
         * The position can be increased no longer than the length of the
         * complete trajectory.
         *
         * @param dt [in] a double that describes how much to increase the
         * iterator position
         */
		virtual void operator +=(double dt) {
			inc(dt);
		}
		

        /**
         * @brief Operator overloading ++ for increasing the position of the iterator.
         *
		 * Usage: ++iterator
		 *
         * The increment is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
		virtual TrajectoryIterator& operator++() {
			inc();
			return *this;
		}


        /**
         * @brief Operator overloading ++ for increasing the position of the iterator.
         *
		 * Usage: iterator++
		 *
         * The increment is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
		virtual void operator++(int) {
			inc();
		}

		/**
         * @brief Operator overloading -- for decreasing the position of the iterator.
		 *
		 * Usage: --iterator;
         *
         * The decrement is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
		virtual TrajectoryIterator& operator--() {
			dec();
			return *this;
		}

		/**
         * @brief Operator overloading -- for decreasing the position of the iterator.
		 *
		 * Usage: iterator--;
         *
         * The decrement is equal to the \b dt specified in the constructor.
         * @return Reference to the TrajectoryIterator
         */
		virtual void operator--(int) {
			dec();
		}

        /**
         * @brief Test if the end of the trajectory is reached.
         *
         * @return true if the iterator has reached the end of the trajectory false
         * otherwise.
         */
        virtual bool isEnd() const = 0;

        /**
         * @brief Test if the beginning of the trajectory is reached.
         *
         * @return true if the iterator has reached the beginning of the trajectory
         * false otherwise.
         */
        virtual bool isBegin() const = 0;

        /**
         * @brief Extracts a point at the current position in the trajectory.
         *
         * @return the point at the current position in the trajectory.
         */
        virtual T operator*() const = 0;

        /**
         * @brief Extracts a point at the current position in the trajectory.
         *
         * @return the point at the current position in the trajectory.
         */        
		virtual T x() const = 0;

        /**
         * @brief Extracts a point of the derivative of the trajectory
         * at the current position in the trajectory
         *
         * @return the derived point at the current position in the trajectory.
         */
        virtual T dx() const = 0;

        /**
         * @brief Extracts a point of the double derivative of the trajectory at the
         * current position in the trajectory
         *
         * @return the double derived point at the current position in the trajectory.
         */
        virtual T ddx() const = 0;

    };

    /** @} */

}} // end namespaces

#endif // end include guard
