#ifndef RW_TRAJECTORY_TRAJECTORY_HPP
#define RW_TRAJECTORY_TRAJECTORY_HPP

/**
 * @file Trajectory.hpp
 */

#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "Interpolator.hpp"
#include "Blend.hpp"



namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/

/**
 * @brief Forward declaration of Trajectory Iterator (needed for friend declaration)
 */
template <class T>
class TrajectoryIterator;

/**
 * @brief Sequence of interpolators and blends giving a trajectory
 *
 * A trajectory is defined as a sequence of interpolators and blends. Multiple interpolators
 * can follow each other, whereas a Blend must be preceded and followed by interpolators.
 *
 * The length of a Trajectory is defined as the time it takes to go from start to finish.
 *
 * When performing random queries the trajectory needs to do a binary search through all
 * interpolators and blend, giving the random access an O(lg n) complexity.
 *
 * For accessing multiple consecutive values use TrajectoryInterpolator.
 *
 * Example of usage:
 * \code
 * Transform3D<> T1(Vector3D<>(0,0,0), EAA<>(0,0,0));
 * Transform3D<> T2(Vector3D<>(1,1,0), EAA<>(1,1,0));
 * Transform3D<> T3(Vector3D<>(2,0,0), EAA<>(2,2,0));
 *
 * LineInterpolator<Transform3D<> >* cartInt1 = new LineInterpolator<Transform3D<> >(T1, T2, 1);
 * LineInterpolator<Transform3D<> >* cartInt2 = new LineInterpolator<Transform3D<> >(T2, T3, 1);
 * ParabolicBlend<Transform3D<> >* blend1 = new ParabolicBlend<Transform3D<> >(cartInt1, cartInt2, 0.25);
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
 * @brief Declares TrajectoryIterator as friend to allow it to use the private parts of Trajectory
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
	 * @param t [in] time between 0 and getLength()
	 * @return Position
	 */
	T x(double t) const {
	    Segment segment = getSegment(t);
	    return getX(segment, t);
	}


    /**
     * @brief Velocity of trajectory at time \b t
     *
     * Returns the velocity of the trajectory at time \b t \f$\in[0,length]\f$.
     *
     * @param t [in] time between 0 and getLength()
     * @return Velocity
     */
	T dx(double t) const {
	    Segment segment = getSegment(t);
	    return getDX(segment, t);
	}



   /**
     * @brief Acceleration of trajectory at time \b t
     *
     * Returns the acceleration of the trajectory at time \b t \f$\in[0,length]\f$.
     *
     * @param t [in] time between 0 and getLength()
     * @return Acceleration
     */
	T ddx(double t) const {
        Segment segment = getSegment(t);
        return getDDX(segment, t);
	}



	/**
	 * @brief Total duration of the trajectory.
	 *
	 * The duration of the Trajectory corresponds to the time it takes to run through it.
	 */
    double duration() const {
        if (_segments.empty())
            return 0;
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
	void add(Interpolator<T>* interpolator) {
	    add(NULL, interpolator);
	}

	/**
	 * @brief Adds a blend and an interpolator to the trajectory.
	 *
	 * The Blend added is used to blend between what was previously the last Interpolator of the
	 * tjajectory onto \b interpolator, which become the new last interpolator of the trajectory.
	 *
	 * Trajectory takes ownership of both \b blend and \b interpolator
	 */
	void add(Blend<T>* blend,
	         Interpolator<T>* interpolator) {
        Segment segment;
        segment.interpolator = boost::shared_ptr<Interpolator<T> >(interpolator);
        segment.blend1 = boost::shared_ptr<Blend<T> >(blend);
        if (_segments.size() > 0) {
            segment.t1 = _segments.back().t2;
            _segments.back().blend2 = segment.blend1;
        } else {
            segment.t1 = 0;
        }
       segment.t2 = segment.t1 + interpolator->duration();
       _segments.push_back(segment);
	}


	/**
	 * @brief Append \b trajectory to the end
	 *
	 * When adding a Trajectory all interpolators and blends of \b trajectory is
	 * added in sequence.
	 *
	 * Ownership of the interpolator and blends are shared using boost::shared_ptr
	 *
	 * @param trajectory [in] Trajectory to append
	 */
	void add(Trajectory<T>* trajectory) {
	    BOOST_FOREACH(const Segment& segment, trajectory->_segments) {
	        Segment newSegment;
	        newSegment.interpolator = segment.interpolator;
	        newSegment.blend1 = segment.blend1;
	        newSegment.blend2 = segment.blend2;
	        if (_segments.size() > 0) {
	            newSegment.t1 = _segments.back().t2;
	            _segments.back().blend2 = newSegment.blend1;
	        } else {
	            newSegment.t1 = 0;
	        }
	       newSegment.t2 = newSegment.t1 + segment.interpolator->duration();

	       _segments.push_back(newSegment);

	    }
	}

private:

    /**
     * @brief Describes a segment consisting of an interpolator and how to blend
     * onto and away from it.
     */
    struct Segment {
        boost::shared_ptr<Blend<T> > blend1;
        boost::shared_ptr<Blend<T> > blend2;
        boost::shared_ptr<Interpolator<T> > interpolator;
        double t1;
        double t2;
    };

    typedef std::vector<Segment> SegmentList;

    SegmentList _segments;

    Segment segmentSearch(double t, double index, double delta) const {
        Segment segment = _segments[(int)index];
        if (segment.t1 > t)
            return segmentSearch(t, index - delta/2.0, delta/2.0);
        else if (segment.t2 < t)
            return segmentSearch(t, index + delta/2.0, delta/2.0);
        else
            return segment;
    }

    Segment getSegment(double t) const {
        //Perform Binary search for the right segment
        size_t n = _segments.size();
        if (n>0) {
            if (_segments.back().t2 < t || t < 0)
                RW_THROW("The requested time is outside the interval of the Trajectory");

            return segmentSearch(t, n/2.0, n/2.0);
        } else {
            RW_THROW("Cannot request values from an empty Trajectory");
        }
    }

    T getX(const typename Trajectory<T>::Segment& segment, double t) const {
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

    T getDX(const Segment& segment, double t) const {
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


    T getDDX(const Segment& segment, double t) const {
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


} //end namespace sandbox
} //end namespace rw


#endif //RW_SANDBOX_TRAJECTORY_HPP
