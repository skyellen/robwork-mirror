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


#ifndef RW_TRAJECTORY_RAMPINTERPOLATOR_HPP
#define RW_TRAJECTORY_RAMPINTERPOLATOR_HPP

/**
 * @file RampInterpolator.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Math.hpp>

#include "Interpolator.hpp"
//#include "InterpolatorUtil.hpp"

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
     * @brief Make a ramp interpolation between two position
     *
     * The template argument given needs to support addition with the "+" operator
     * and scaling with a double using the "*" operator.
     *
     * For use with a rw::math::Transform3D see the template specialization
     */
    template <class T>
    class RampInterpolator: public Interpolator<T>
    {
    public:
		//! @brief smart pointer type to instance of class
		typedef typename rw::common::Ptr<RampInterpolator> Ptr;

		//! @brief smart pointer type const instance of class
		typedef typename rw::common::Ptr<const RampInterpolator> CPtr;

        /**
         * @brief Construct RampInterpolator starting at \b start and finishing in \b end
         * with velocity limits \b vellimimts and acceleration limits \b acclimits. The duration
         * will be calculated automatically. The start and end velocity and acceleration is zero.
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param vellimits [in] velocity limits
         * @param acclimits [in] acceleration limits
         */
        RampInterpolator(const T& start, const T& end, const T& vellimits, const T& acclimits) :
            _a(start),
            _b(end - start),
            _vel(vellimits),
            _acc(acclimits),
            _duration(-1.0)

        {
            init();
        }

        /**
         * @brief Construct RampInterpolator starting at \b start and finishing in \b end
         * with velocity limits \b vellimimts and acceleration limits \b acclimits. The duration
         * will be calculated automatically. The start and end velocity and acceleration is zero.
         *
         * If \b duration is not achievable given the velocity and acceleration limits then
         * the duration will be extended.
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param vellimits [in] velocity limits
         * @param acclimits [in] acceleration limits
         * @param duration [in] Time it takes to from one end to the other.
         */
        RampInterpolator(const T& start, const T& end, const T& vellimits, const T& acclimits, double duration) :
            _a(start),
            _b(end - start),
            _vel(vellimits),
            _acc(acclimits),
            _duration(duration)
        {
            init();
        }


        void init(){
            using namespace rw::math;
            // calculate max time
            double maxtime = 0;
            for (size_t i = 0; i<_b.size(); i++) {
                double t = 0;
                double tau = _vel(i)/_acc(i);
                double eps = sqrt(fabs(_b(i))/(_acc(i)));
                //std::cout<<"tau = "<<tau<<std::endl;
                //std::cout<<"eps = "<<eps<<std::endl;
                if (eps<tau)
                    t = 2*eps;
                else {
                    double dtau = tau*_vel(i);
                    double Ttmp = (fabs(_b(i))-dtau)/_vel(i);
                    t = Ttmp+2*tau;
                }
                maxtime = std::max(t, maxtime);
            }
            _duration = maxtime;

            // calculate ramp
            _wmax = 1e10;
            _dwmax = 1e10;
            _ws = 0;

            for (size_t i = 0; i<_a.size(); i++) {
                //double dq = fabs(_qend(i)-_qstart(i));
                double dq = fabs(_b(i));
                //std::cout<<"dq = "<<dq<<std::endl;
                if (dq != 0) {
                    _wmax = std::min(_wmax, _vel(i)/dq);
                    _dwmax = std::min(_dwmax, _acc(i)/dq);
                }
            }
            //std::cout<<"wmax = "<<_wmax<<std::endl;
            //std::cout<<"dwmax = "<<_dwmax<<std::endl;

            //Compare using eq. 13 to see which of the cases
            double lhs = sqrt(2*Math::sqr(_ws/_dwmax)+4/_dwmax);
            double rhs = 2*_wmax/_dwmax;
            if (lhs <= rhs) {
                _durationRamp = _ws/_wmax+sqrt(2*Math::sqr(_ws/_dwmax)+4/_dwmax);
                _tau_s = _durationRamp/2-_ws/(2*_dwmax);
                _tau_e = _durationRamp/2+_ws/(2*_dwmax);
            } else {
                _durationRamp = 1/_wmax+(Math::sqr(_wmax-_ws)+Math::sqr(_wmax))/(2*_wmax*_dwmax);
                _tau_s = (_wmax-_ws)/_dwmax;
                _tau_e = (_wmax)/_dwmax;
            }


        }

        /**
         * @copydoc Interpolator::x()
         */
        T x(double t) const
        {
            if (t<0)
                t = 0;

            double tau1 = std::min(t, _tau_s);
            double s1 = _ws*t+0.5*_dwmax*tau1*tau1;
            if (t<_tau_s)
                return _a+s1*_b;

            double tau2 = std::min(t, _durationRamp-_tau_e);
            double s2 = _wmax*(tau2-_tau_s) + s1;
            if (t<=_durationRamp-_tau_e)
                return _a+s2*_b;

            if (t<_durationRamp) {
                double delta = (_duration-_tau_e);
                double tpart = _tau_e*_dwmax*t + 0.5*_dwmax*t*t-_dwmax*_durationRamp*t;
                double cpart = _tau_e*_dwmax*delta + 0.5*_dwmax*delta*delta - _dwmax*_durationRamp*delta;

                return _a+_b*(s2 - (tpart - cpart)+_tau_e*_dwmax*t-_tau_e*_dwmax*delta);
            }
            return _a+_b;
        }

        /**
         * @copydoc Interpolator::dx()
         */
        T dx(double t) const
        {
            if (t<0)
                t = 0;

            double tau1 = std::min(t, _tau_s);
            double ds1 = _ws+_dwmax*tau1;
            if (t<_tau_s)
                return ds1*_b;

            // double tau2 = std::min(t, _duration-_tau_e);
            double ds2 = ds1;
            if (t<=_durationRamp-_tau_e)
                return ds2*_b;

            if (t<_durationRamp) {
                double delta = t-(_durationRamp-_tau_e);
                double ds3 = ds2 - _dwmax*delta;
                return ds3*_b;
            }
            return _b*0.0;
        };

        /**
         * @copydoc Interpolator::ddx()
         */
        T ddx(double t) const
        {
            if (t<_tau_s)
                return _dwmax*_b;
            if (t<_durationRamp-_tau_e)
                return 0*_b;
            if (t<_durationRamp)
                return -_dwmax*_b;
            return 0*_b;
        }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        T getStart() const { return _a; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        T getEnd() const { return _a + _b; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _durationRamp; }

    private:
        T _a;
        T _b;
        T _vel;
        T _acc;
        double _duration; // maxtime

        //double _maxtime;

        double _durationRamp;
        double _ws;
        double _wmax;
        double _dwmax;
        double _tau_s;
        double _tau_e;
    };


    template <>
    class RampInterpolator<double> : public Interpolator<double>
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RampInterpolator<double> > Ptr;

        //! @brief smart pointer type const instance of class
        typedef rw::common::Ptr<const RampInterpolator<double> > CPtr;

        /**
         * @brief Construct RampInterpolator starting a \b start and finishing in \b end.
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param velLimit [in] the max velocity in m/sec
         * @param accLimit [in] the max acceleration in m/sec^2
         */
        RampInterpolator(double start, double end, double velLimit,double accLimit):
            _start( start ),
            _end( end ),
            _ramp(rw::math::Q(1,0.0), rw::math::Q(1,_end), rw::math::Q(1,velLimit), rw::math::Q(1,accLimit) )
        {
        }

        virtual ~RampInterpolator(){}

        //! @copydoc Interpolator::x()
        double x(double t) const { return _ramp.x(t)(0); }

        //! @copydoc Interpolator::dx()
        double dx(double t) const { return _ramp.dx(t)(0); }

        //! @copydoc Interpolator::ddx()
        double ddx(double t) const { return _ramp.ddx(t)(0); }

        /**
         * @brief Returns the start rotation of the interpolator
         * @return The start rotation of the interpolator
         */
        double getStart() const { return _start; }

        /**
         * @brief Returns the end rotation of the interpolator
         * @return The end rotation of the interpolator
         */
        double getEnd() const { return _end; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _ramp.duration(); }

    private:
        double _start;
        double _end;
        //double _b;
        RampInterpolator<rw::math::Q> _ramp;

        //double _duration;
    };


    /**
     * @brief Forward declaration for parabolic blend to make the
     * RampInterpolator<rw::math::Transform3D<T> > a friend
     */
    template <class T>
    class ParabolicBlend;


    /**
     * @brief Implements RampInterpolator for rw::math::Rotation3D<T>
     *
     * The interpolation of rotation is made using a Quaternion slerp interpolation.
     * See rw::math::Quaternion::slerp for further information.
     *
     */
    template <class T>
    class RampInterpolator<rw::math::Rotation3D<T> > : public Interpolator<rw::math::Rotation3D<T> >
    {
    public:
		//! @brief smart pointer type to this class
		typedef typename rw::common::Ptr<RampInterpolator<rw::math::Rotation3D<T> > > Ptr;
		
		//! @brief smart pointer type const instance of class
		typedef typename rw::common::Ptr<const RampInterpolator<rw::math::Rotation3D<T> > > CPtr;

        /**
         * @brief Construct RampInterpolator starting a \b start and finishing in \b end.
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param velLimit [in] the max rotational velocity in rad/sec
         * @param accLimit [in] the max rotational acceleration in rad/sec^2
         */
        RampInterpolator(const rw::math::Rotation3D<T>& start,
                           const rw::math::Rotation3D<T>& end,
                           double velLimit,
                           double accLimit):
            _start( start ),
            _end( end ),
            _b( inverse(start)*end ),
            _ramp(rw::math::Q(1,0.0), rw::math::Q(1,_b.angle()), rw::math::Q(1,velLimit), rw::math::Q(1,accLimit) )
        {
            double angle = _b.angle();
            if(angle>0.000001){
                _b[0] = _b[0]/angle;
                _b[1] = _b[1]/angle;
                _b[2] = _b[2]/angle;
            }
        }

        /**
         * @copydoc Interpolator::x()
         */
        rw::math::Rotation3D<T> x(double t) const
        {
            rw::math::Rotation3D<T> srot= _start.toRotation3D();
            double scale = _ramp.x(t)(0);
            rw::math::EAA<T> rot(_b[0]*scale, _b[1]*scale, _b[2]*scale);
            return srot* rot.toRotation3D();
        }

        /**
         * @copydoc Interpolator::dx()
         */
        rw::math::Rotation3D<T> dx(double t) const
        {
            double scale = _ramp.dx(t)(0);
            rw::math::EAA<T> rot(_b[0]*scale, _b[1]*scale, _b[2]*scale);
            return rot.toRotation3D();
        }

        /**
         * @copydoc Interpolator::ddx()
         */
        rw::math::Rotation3D<T> ddx(double t) const
        {
            double scale = _ramp.ddx(t)(0);
            rw::math::EAA<T> rot(_b[0]*scale, _b[1]*scale, _b[2]*scale);
            return rot.toRotation3D();
        }

        /**
         * @brief Returns the start rotation of the interpolator
         * @return The start rotation of the interpolator
         */
        rw::math::Rotation3D<T> getStart() const { return _start.toRotation3D(); }

        /**
         * @brief Returns the end rotation of the interpolator
         * @return The end rotation of the interpolator
         */
        rw::math::Rotation3D<T> getEnd() const { return _end.toRotation3D(); }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _ramp.duration(); }

    private:
        rw::math::EAA<T> _start;
        rw::math::EAA<T> _end;
        rw::math::EAA<T> _b;
        RampInterpolator<rw::math::Q> _ramp;

        double _duration;
    };

    /**
     * @brief Implements RampInterpolator for rw::math::Vector3D<T>
     *
     * The interpolation of Vector3D<> is done over the cartsean distance so that the
     * limits are defined in cartesean space and not on the individual axis of Vector3D
     */
    template <class T>
    class RampInterpolator<rw::math::Vector3D<T> > : public Interpolator<rw::math::Vector3D<T> >
    {
    public:
        //! @brief smart pointer type to this class
        typedef typename rw::common::Ptr<RampInterpolator<rw::math::Vector3D<T> > > Ptr;

        //! @brief smart pointer type const instance of class
        typedef typename rw::common::Ptr<const RampInterpolator<rw::math::Vector3D<T> > > CPtr;

        /**
         * @brief Construct RampInterpolator starting a \b start and finishing in \b end.
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param velLimit [in] the max velocity in m/sec
         * @param accLimit [in] the max acceleration in m/sec^2
         */
        RampInterpolator(const rw::math::Vector3D<T>& start,
                           const rw::math::Vector3D<T>& end,
                           double velLimit,
                           double accLimit):
            _start( start ),
            _end( end ),
            _b( end-start ),
            _ramp(rw::math::Q(1,0.0), rw::math::Q(1,_b.norm2()), rw::math::Q(1,velLimit), rw::math::Q(1,accLimit) )
        {
        }

        /**
         * @copydoc Interpolator::x()
         */
        rw::math::Vector3D<T> x(double t) const
        {
            return _start + normalize(_b)*_ramp.x(t)(0);
        }

        /**
         * @copydoc Interpolator::dx()
         */
        rw::math::Vector3D<T> dx(double t) const
        {
            return normalize(_b)*_ramp.dx(t)(0);
        }

        /**
         * @copydoc Interpolator::ddx()
         */
        rw::math::Vector3D<T> ddx(double t) const
        {
            return normalize(_b)*_ramp.ddx(t)(0);
        }

        /**
         * @brief Returns the start rotation of the interpolator
         * @return The start rotation of the interpolator
         */
        rw::math::Vector3D<T> getStart() const { return _start; }

        /**
         * @brief Returns the end rotation of the interpolator
         * @return The end rotation of the interpolator
         */
        rw::math::Vector3D<T> getEnd() const { return _end; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _ramp.duration(); }

    private:
        rw::math::Vector3D<T> _start;
        rw::math::Vector3D<T> _end;
        rw::math::Vector3D<T> _b;
        RampInterpolator<rw::math::Q> _ramp;

        double _duration;
    };


    /**
     * @brief Implements RampInterpolator for rw::math::Transform3D<T>
     *
     * The interpolation of rotation is made using Quaternions.
     */
    template <class T>
    class RampInterpolator<rw::math::Transform3D<T> > : public Interpolator<rw::math::Transform3D<T> >
    {
     public:
		 //! @brief smart pointer type to this class
		typedef typename rw::common::Ptr<RampInterpolator<rw::math::Transform3D<T> > > Ptr;

		//! @brief smart pointer type const instance of class
		typedef typename rw::common::Ptr<const RampInterpolator<rw::math::Transform3D<T> > > CPtr;

        /**
         * @brief Construct RampInterpolator starting a \b start and finishing in \b end
         * and taking \b duration time.
         *
         * If \b duration <= 0 an exception is thrown
         *
         * @param start [in] Start of interpolator
         * @param end [in] End of interpolator
         * @param duration [in] Time it takes to from one end to the other.
         */
        RampInterpolator(const rw::math::Transform3D<T>& start,
                           const rw::math::Transform3D<T>& end,
                           double linVelLimit,
                           double linAccLimit,
                           double angVelLimit,
                           double angAccLimit):
            _start(start),
            _end(end),
            _posInterpolator(start.P(), end.P(), linVelLimit, linAccLimit),
            _rotInterpolator(start.R(), end.R(), angVelLimit, angAccLimit)
        {
            _duration = std::max(_posInterpolator.duration(), _rotInterpolator.duration() );
        }

        /**
         * @copydoc Interpolator::x()
         */
        rw::math::Transform3D<T> x(double t) const
        {
            return rw::math::Transform3D<T>(_posInterpolator.x(t), _rotInterpolator.x(t));
            //std::cout<<"Pose = "<<_interpolator.x(t)<<std::endl;
            //return InterpolatorUtil::vecToTrans<V,T>(_interpolator.x(t));
        }

        /**
         * @copydoc Interpolator::dx()
         */
        rw::math::Transform3D<T> dx(double t) const
        {
            return rw::math::Transform3D<T>(_posInterpolator.dx(t), _rotInterpolator.dx(t));
            //return InterpolatorUtil::vecToTrans<V,T>(_interpolator.dx(t));
        }

        /**
         * @copydoc Interpolator::ddx()
         */
        rw::math::Transform3D<T> ddx(double t) const
        {
            return rw::math::Transform3D<T>(_posInterpolator.ddx(t), _rotInterpolator.ddx(t));
            //return InterpolatorUtil::vecToTrans<V,T>(_interpolator.ddx(t));
        }

        /**
         * @brief Returns the start position of the interpolator
         * @return The start position of the interpolator
         */
        rw::math::Transform3D<T> getStart() const { return _start; }

        /**
         * @brief Returns the end position of the interpolator
         * @return The end position of the interpolator
         */
        rw::math::Transform3D<T> getEnd() const { return _end; }

        /**
         * @copydoc Interpolator::duration()
         */
        double duration() const { return _duration; }


        /**
         * @brief Returns RampInterpolator for the position part of the transform
         *
         * @note This method is needed by ParabolicBlend
         */
		const RampInterpolator<rw::math::Vector3D<T> >& getPositionInterpolator() const {
            return _posInterpolator;
        }

        /**
         * @brief Returns RampInterpolator for the rotation part of the transform
         *
         * @note This method is needed by ParabolicBlend
         */
		const RampInterpolator<rw::math::Rotation3D<T> >& getRotationInterpolator() const {
            return _rotInterpolator;
        }

    private:
        rw::math::Transform3D<T> _start;
        rw::math::Transform3D<T> _end;
        RampInterpolator<rw::math::Vector3D<> > _posInterpolator;
        RampInterpolator<rw::math::Rotation3D<> > _rotInterpolator;
        double _duration;
    };

    /**
     * @brief RampInterpolator with T=rw:math::Q
     */
    typedef RampInterpolator<rw::math::Q> QRampInterpolator;

    /**
     * @brief RampInterpolator with T=rw:math::Transform3D<>
     */
    typedef RampInterpolator<rw::math::Transform3D<> > CartesianRampInterpolator;

    /** @} */

}} // end namespaces

#endif // end include guard
