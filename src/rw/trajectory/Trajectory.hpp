#ifndef RW_TRAJECTORY_TRAJECTORY_HPP
#define RW_TRAJECTORY_TRAJECTORY_HPP

/**
   @file Trajectory.hpp
*/

#include <rw/common/macros.hpp>
#include <rw/common/Ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "Interpolator.hpp"
#include "Blend.hpp"

namespace rw { namespace math { class Q; }}
namespace rw { namespace kinematics { class State; }}

namespace rw { namespace trajectory {

	/** @addtogroup trajectory */
	/*@{*/

    template <class T> class Trajectory;

    //! A trajectory on rw::kinematics::State.
    typedef Trajectory<rw::kinematics::State> StateTrajectory;

    //! A pointer to a StateTrajectory.
    typedef rw::common::Ptr<StateTrajectory> StateTrajectoryPtr;

    //! A trajectory on rw::math::Q.
    typedef Trajectory<rw::math::Q> QTrajectory;

    //! A pointer to a QTrajectory.
    typedef rw::common::Ptr<QTrajectory> QTrajectoryPtr;

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
     * LineInterpolator<Transform3D<> >* cartInt1 =
     *     new LineInterpolator<Transform3D<> >(T1, T2, 1);
     * LineInterpolator<Transform3D<> >* cartInt2 =
     *     new LineInterpolator<Transform3D<> >(T2, T3, 1);
     * ParabolicBlend<Transform3D<> >* blend1 =
     *   new ParabolicBlend<Transform3D<> >(cartInt1, cartInt2, 0.25);
     * Trajectory<Transform3D<> > trajectory;
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
    class Trajectory
    {
        /**
           @brief Declares TrajectoryIterator as friend to allow it to use the
           private parts of Trajectory.
        */
        friend class TrajectoryIterator<T>;

    public:
        /**
         * @brief Construct an empty trajectory
         */
        Trajectory() {}

        /**
         * @brief Destructor
         */
        virtual ~Trajectory() {}

        /**
         * @brief Position of trajectory at time \b t
         *
         * Returns the position of the trajectory at time \b t \f$\in[0,length]\f$.
         *
         * @param t [in] time between 0 and duration()
         * @return Position
         */
        T x(double t) const
        {
            const Segment& segment = getSegment(t);
            return getX(segment, t);
        }

        /**
         * @brief Velocity of trajectory at time \b t
         *
         * Returns the velocity of the trajectory at time \b t \f$\in[0,length]\f$.
         *
         * @param t [in] time between 0 and duration()
         * @return Velocity
         */
        T dx(double t) const
        {
            const Segment& segment = getSegment(t);
            return getDX(segment, t);
        }

       /**
         * @brief Acceleration of trajectory at time \b t
         *
         * Returns the acceleration of the trajectory at time \b t \f$\in[0,length]\f$.
         *
         * @param t [in] time between 0 and duration()
         * @return Acceleration
         */
        T ddx(double t) const
        {
            const Segment& segment = getSegment(t);
            return getDDX(segment, t);
        }

        /**
           @brief Total duration of the trajectory.

           The duration of the Trajectory corresponds to the time it takes to
           run through it.

           If the trajectory is empty, then -1 is returned.
        */
        double duration() const
        {
            if (_segments.empty())
                return -1;
            else
                return _segments.back().t2;
        }

        /**
         * @brief Adds an interpolator to the end of the trajectory.
         *
         * When adding the interpolator the Trajectory takes ownership.
         *
         * @param interpolator [in] The interpolator to add
         */
        void add(Interpolator<T>* interpolator) { add(NULL, interpolator); }

        /**
         * @brief Adds a blend and an interpolator to the trajectory.
         *
         * The Blend added is used to blend between what was previously the last
         * Interpolator of the tjajectory onto \b interpolator, which become the
         * new last interpolator of the trajectory.
         *
         * Trajectory takes ownership of both \b blend and \b interpolator
         */
        void add(
            Blend<T>* blend,
            Interpolator<T>* interpolator)
        {
            addSegment(blend_ptr(blend), blend_ptr(), interpolator_ptr(interpolator));
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
        void add(Trajectory<T>* trajectory)
        {
            BOOST_FOREACH(const Segment& segment, trajectory->_segments) {
                addSegment(segment.blend1, segment.blend2, segment.interpolator);
            }
        }

    private:
        typedef boost::shared_ptr<Blend<T> > blend_ptr;
        typedef boost::shared_ptr<Interpolator<T> > interpolator_ptr;

        /**
         * @brief Describes a segment consisting of an interpolator and how to blend
         * onto and away from it.
         */
        struct Segment
        {
            Segment(
                blend_ptr blend1,
                blend_ptr blend2,
                interpolator_ptr interpolator,
                double t1,
                double t2)
                :
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

            blend_ptr blend1;
            blend_ptr blend2;
            interpolator_ptr interpolator;
            double t1;
            double t2;
        };

        typedef std::vector<Segment> SegmentList;
        SegmentList _segments;

        void addSegment(blend_ptr blend1, blend_ptr blend2, interpolator_ptr interpolator)
        {
            const double t1 = _segments.empty() ? 0 : duration();
            RW_ASSERT(t1 >= 0);

            RW_ASSERT(interpolator->duration() >= 0);
            const double t2 = t1 + interpolator->duration();

            RW_ASSERT(t2 >= 0);

            const Segment segment(
                blend1,
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
            if (n > 0) {
                if (t < 0 || _segments.back().t2 < t)
                    RW_THROW(
                        "The requested time is outside the interval of the trajectory.\n"
                        "t: " << t << " end: " << duration() << " cnt: " << (int)_segments.size()
                        );

                return segmentSearch(t, n/2.0, n/2.0);
            } else {
                RW_THROW("Cannot request values from an empty Trajectory");
            }
        }

        T getX(const typename Trajectory<T>::Segment& segment, double t) const
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
    };

    /** @} */

}} // end namespaces

#endif // end include guard
