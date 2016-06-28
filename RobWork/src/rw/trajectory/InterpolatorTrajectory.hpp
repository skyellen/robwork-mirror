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


#ifndef RW_TRAJECTORY_INTERPOLATORTRAJECTORY_HPP
#define RW_TRAJECTORY_INTERPOLATORTRAJECTORY_HPP

/**
   @file InterpolatorTrajectory.hpp
*/


#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>
#include <boost/foreach.hpp>

#include "Trajectory.hpp"
#include "Interpolator.hpp"
#include "Blend.hpp"

namespace rw { namespace math { class Q; }}
namespace rw { namespace kinematics { class State; }}

namespace rw { namespace trajectory {

	/** @addtogroup trajectory */
	/*@{*/


    /*template <class T> class Trajectory;

    //! A trajectory on rw::kinematics::State.
    typedef Trajectory<rw::kinematics::State> StateTrajectory;

    //! A pointer to a StateTrajectory.
    typedef rw::common::Ptr<StateTrajectory> StateTrajectoryPtr;
*/

    /**
     * @brief Forward declaration of Trajectory Iterator (needed for friend
     * declaration)
     */
    template <class T>
    class TrajectoryIterator;

    /**
     * @brief Sequence of interpolators and blends giving a trajectory
     *
     * A trajectory is defined as a sequence of interpolators and blends.
     * Multiple interpolators can follow each other, whereas a Blend must be
     * preceded and followed by interpolators.
     *
     * The length of a Trajectory is defined as the time it takes to go from
     * start to finish.
     *
     * When performing random queries the trajectory needs to do a binary search
     * through all interpolators and blend, giving the random access an O(lg n)
     * complexity.
     *
     * For accessing multiple consecutive values use TrajectoryInterpolator.
     *
     * Example of usage:
     * \code
     * Transform3D<> T1(Vector3D<>(0,0,0), EAA<>(0,0,0));
     * Transform3D<> T2(Vector3D<>(1,1,0), EAA<>(1,1,0));
     * Transform3D<> T3(Vector3D<>(2,0,0), EAA<>(2,2,0));
     *
     * LinearInterpolator<Transform3D<> >::Ptr cartInt1 =
     *     ownedPtr(new LinearInterpolator<Transform3D<> >(T1, T2, 1));
     * LinearInterpolator<Transform3D<> >::Ptr cartInt2 =
     *     ownedPtr(new LinearInterpolator<Transform3D<> >(T2, T3, 1));
     * ParabolicBlend<Transform3D<> >::Ptr blend1 =
     *     ownedPtr(new ParabolicBlend<Transform3D<> >(cartInt1, cartInt2, 0.25));
     * InterpolatorTrajectory<Transform3D<> > trajectory;
     * trajectory.add(cartInt1);
     * trajectory.add(blend1, cartInt2);
     * std::ofstream out("test.dat");
     * for (double t = 0; t<=trajectory.duration(); t += dt) {
     *      Transform3D<> x = trajectory.x(t);
     *      out<<t<<" "<<x.P()(0)<<" "<<x.P()(1)<<" "<<x.P()(2)<<std::endl;
     * }
     * out.close();
     * \endcode
     */
    template <class T>
    class InterpolatorTrajectory: public Trajectory<T>
    {
        /**
           @brief Declares TrajectoryIterator as friend to allow it to use the
           private parts of Trajectory.
        */
        friend class TrajectoryIterator<T>;

    public:
        //! @brief smart pointer type
        typedef rw::common::Ptr<InterpolatorTrajectory<T> > Ptr;

        /**
         * @brief Construct an empty trajectory
         */
        InterpolatorTrajectory(double startTime = 0):
            _startTime(startTime)
        {
        }

        /**
         * @brief Destructor
         */
        virtual ~InterpolatorTrajectory() {}



        /**
         * @copydoc Trajectory::x
         */
        T x(double t) const
        {
            t -= startTime();
            const Segment& segment = getSegment(t);
            return getX(segment, t);
        }

        /**
         * @copydoc Trajectory::dx
         */
        T dx(double t) const
        {
            t -= startTime();
            const Segment& segment = getSegment(t);
            return getDX(segment, t);
        }

        /**
         * @copydoc Trajectory::ddx
         */
        T ddx(double t) const
        {
            t -= startTime();
            const Segment& segment = getSegment(t);
            return getDDX(segment, t);
        }

        /**
         * @copydoc Trajectory::duration
         */
        double duration() const
        {
            if (_segments.empty())
                return -1;
            else
                return _segments.back().t2;
        }

        /**
         * @copydoc Trajectory::startTime()
         */
        double startTime() const {
            return _startTime;
        }

		/**
		 * @copydoc Trajectory::getIterator
		 */
		typename TrajectoryIterator<T>::Ptr getIterator(double dt) const {
			return rw::common::ownedPtr(new InterpolatorTrajectoryIterator<T>(const_cast<InterpolatorTrajectory*>(this), dt));
		}



        /**
         * @brief Adds an interpolator to the end of the trajectory.
         *
         * When adding the interpolator the Trajectory takes ownership.
         *
         * @param interpolator [in] The interpolator to add
         */
        void add(rw::common::Ptr<Interpolator<T> > interpolator) {
            add(NULL, interpolator);
        }

        /**
         * @brief Adds a blend and an interpolator to the trajectory.
         *
         * The Blend added is used to blend between what was previously the last
         * Interpolator of the trajectory onto \b interpolator, which become the
         * new last interpolator of the trajectory.
         */
        void add(rw::common::Ptr<Blend<T> > blend,
                 rw::common::Ptr<Interpolator<T> > interpolator)
        {
            addSegment(blend, rw::common::Ptr<Blend<T> >(NULL), interpolator);
        }

        /**
         * @brief Append \b trajectory to the end
         *
         * When adding a Trajectory all interpolators and blends of \b
         * trajectory is added in sequence.
         *
         * Ownership of the interpolator and blends are shared using
         * boost::shared_ptr
         *
         * @param trajectory [in] Trajectory to append
         */
        void add(InterpolatorTrajectory<T>* trajectory)
        {
            BOOST_FOREACH(const Segment& segment, trajectory->_segments) {
                addSegment(segment.blend1, segment.blend2, segment.interpolator);
            }
        }

        /**
         * @brief Returns the number of segments
         *
         * A segment contains a description interpolator and the blend used to blend from the previous interpolator
         */
        size_t getSegmentsCount() const {
            return _segments.size();
        }

        std::pair<rw::common::Ptr<Blend<T> >, rw::common::Ptr<Interpolator<T> > > getSegment(size_t index) const {
            const Segment segment = _segments[index];
            const rw::common::Ptr<Interpolator<T> > interpolator = segment.interpolator;
            const rw::common::Ptr<Blend<T> > blend = segment.blend1;

            return  std::pair<rw::common::Ptr<Blend<T> >,rw::common::Ptr<Interpolator<T> > >(blend, interpolator);
        }


    private:
#ifdef RW_USE_DEPRECATED
        typedef rw::common::Ptr<Blend<T> > BlendTPtr;
        typedef rw::common::Ptr<Interpolator<T> > InterpolatorTPtr;
#endif
        /**
         * @brief Describes a segment consisting of an interpolator and how to blend
         * onto and away from it.
         */
        struct Segment
        {
			Segment(typename Blend<T>::Ptr blend1,
				typename Blend<T>::Ptr blend2,
				typename Interpolator<T>::Ptr interpolator,
                    double t1,
                    double t2) :
                blend1(blend1),
                blend2(blend2),
                interpolator(interpolator),
                t1(t1),
                t2(t2)
            {
                RW_ASSERT(t1 >= 0);
                RW_ASSERT(t2 >= 0);
                RW_ASSERT(t1 <= t2);
            }

			typename Blend<T>::Ptr blend1;
			typename Blend<T>::Ptr blend2;
			typename Interpolator<T>::Ptr interpolator;
            double t1;
            double t2;
        };

        double _startTime;

        typedef std::vector<Segment> SegmentList;
        SegmentList _segments;

		void addSegment(typename Blend<T>::Ptr blend1, typename Blend<T>::Ptr blend2, typename Interpolator<T>::Ptr interpolator)
        {
            const double t1 = _segments.empty() ? 0 : duration();
            RW_ASSERT(t1 >= 0);

            RW_ASSERT(interpolator->duration() >= 0);
            const double t2 = t1 + interpolator->duration();

            RW_ASSERT(t2 >= 0);

            const Segment segment(blend1,
                                  blend2,
                                  interpolator,
                                  t1,
                                  t2);

            if (!_segments.empty()) {
                _segments.back().blend2 = segment.blend1;
            }

            _segments.push_back(segment);

            RW_ASSERT(_segments.front().t1 == 0);
            RW_ASSERT(_segments.back().t2 >= 0);
        }

        const Segment& segmentSearch(double t, double index, double delta) const
        {
            const Segment& segment = _segments[(int)index];
            if (segment.t1 > t)
                return segmentSearch(t, index - delta/2.0, delta/2.0);
            else if (segment.t2 < t)
                return segmentSearch(t, index + delta/2.0, delta/2.0);
            else
                return segment;
        }

        const Segment& getSegment(double t) const
        {
            // Perform Binary search for the right segment
            const size_t n = _segments.size();
            if( n<1 ){
                RW_THROW("Cannot request values from an empty Trajectory");
            }

            if(t<0) {
            	t=0.0;
            } else if(_segments.back().t2 < t){
            	t=_segments.back().t2;
            }
            /*
            if (t < 0 || _segments.back().t2 < t){
                RW_THROW(
                    "The requested time is outside the interval of the trajectory.\n"
                    "t: " << t << " end: " << duration() << " cnt: " << (int)_segments.size()
                    );
            }
            */

            return segmentSearch(t, n/2.0, n/2.0);
        }

        T getX(const typename InterpolatorTrajectory<T>::Segment& segment, double t) const
        {
            if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
                t = t - segment.t1;
                return segment.blend1->x(t + segment.blend1->tau1());
            } else if (segment.blend2 != NULL &&
                segment.t2 - t < segment.blend2->tau1()) {
                t = t - segment.t2 + segment.blend2->tau1();
                return segment.blend2->x(t);
            } else {
                t = t - segment.t1;
                return segment.interpolator->x(t);
            }
        }

        T getDX(const Segment& segment, double t) const
        {
            if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
                t = t - segment.t1;
                return segment.blend1->dx(t + segment.blend1->tau1());
            } else if (segment.blend2 != NULL &&
                segment.t2 - t < segment.blend2->tau1()) {
                t = t - segment.t2 + segment.blend2->tau1();
                return segment.blend2->dx(t);
            } else {
                t = t - segment.t1;
                return segment.interpolator->dx(t);
            }
        }

        T getDDX(const Segment& segment, double t) const
        {
            if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
                t = t - segment.t1;
                return segment.blend1->ddx(t + segment.blend1->tau1());
            } else if (segment.blend2 != NULL &&
                segment.t2 - t < segment.blend2->tau1()) {
                t = t - segment.t2 + segment.blend2->tau1();
                return segment.blend2->ddx(t);
            } else {
                t = t - segment.t1;
                return segment.interpolator->ddx(t);
            }
        }

		/**
		 * @brief Bi-directional iterator for running efficiently through a trajectory
		 */
		template <class U>
		class InterpolatorTrajectoryIterator: public TrajectoryIterator<U>
		{
		public:
			/**
			 * @brief Constructs iterator for \b trajectory
			 *
			 * @param trajectory [in] Trajectory to iterate through
			 * @param dt [in] Default stepsize used for ++ and -- operators
			 */
			InterpolatorTrajectoryIterator(typename InterpolatorTrajectory<U>::Ptr trajectory, double dt = 1)
			{
				_trajectory = trajectory;
				_dt = dt;
				_time = _trajectory->startTime();
				_currentSegment = trajectory->_segments.begin();
			}

			/**
			 * @copydoc TrajectoryIterator::getTime()
			 */
			double getTime() const { return _time; }

			/**
			 * @copydoc TrajectoryIterator::dec(double)
			 */
			void dec(double dt)
			{
				if (_time - dt < _trajectory->startTime())
					_time = _trajectory->startTime();
				else
					_time -= dt;
				while (_time < _currentSegment->t1 )
					_currentSegment--;
			}

			/**
			 * @copydoc TrajectoryIterator::inc(double)
			 */
			void inc(double dt)
			{
				if (_time + dt > _trajectory->endTime())
					_time = _trajectory->endTime();
				else
					_time += dt;
				while (_time > _currentSegment->t2 )
					_currentSegment++;
			}

			/**
			 * @copydoc TrajectoryIterator::dec()
			 */
			void dec()
			{
				dec(_dt);
			}

			/**
			 * @copydoc TrajectoryIterator::inc()
			 */
			void inc()
			{
				inc(_dt);
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
			U operator*() const { return x(); }

			/**
			 * @copydoc TrajectoryIterator::x()
			 */
			U x() const {
				return _trajectory->getX(*_currentSegment, _time);
			}

			/**
			 * @copydoc TrajectoryIterator::dx()
			 */
			U dx() const {
				return _trajectory->getDX(*_currentSegment, _time);
			}

			/**
			 * @copydoc TrajectoryIterator::ddx()
			 */
			U ddx() const {
				return _trajectory->getDDX(*_currentSegment, _time);
			}

		private:
			typename InterpolatorTrajectory<U>::SegmentList::const_iterator _currentSegment;
			typename InterpolatorTrajectory<U>::Ptr _trajectory;
			double _time;
			double _dt;
		};

    };

    /**
     * @brief InterpolatorTrajectory with type Q
     */
     typedef InterpolatorTrajectory<rw::math::Q> QInterpolatorTrajectory;

#ifdef RW_USE_DEPRECATED
    /**
     * @brief Pointer to QInterpolatorTrajectory
     */
    typedef rw::common::Ptr<QInterpolatorTrajectory> QInterpolatorTrajectoryPtr;
#endif
    /** @} */

}} // end namespaces

#endif // end include guard
