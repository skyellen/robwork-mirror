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

#ifndef RW_TRAJECTORY_BLENDEDTRAJECTORY_HPP
#define RW_TRAJECTORY_BLENDEDTRAJECTORY_HPP

// RW
#include "Trajectory.hpp"
#include "Path.hpp"

#include <rw/models/Device.hpp>
#include <rw/common/macros.hpp>

// STL
#include <vector>
#include <cmath>
#include <limits>

namespace rw { 
namespace trajectory {
    
    /**
     * @brief Implements a trajectory with blends between segments.
     * TODO: Briefly describe how 
     *
     *
     */
    template <class T = rw::math::Q>
    class BlendedTrajectory : public Trajectory<T> {
    public:
        /**
         * @brief Default constructor creating an empty trajectory
         */
        BlendedTrajectory() {};

        /**
         * @brief Constrcuts a trajectory.
         *
         * Constructs a trajectory for device \b deviceIn. Points on the path is described by \b pathIn 
         * and the blend distances in \b betaIn. The number of blend distances has to be path length -2
         *
         * To adjust the velocity and acceleration of the robot use \b vscaleIn and \b ascaleIn
         *
         * @param deviceIn [in] Device to create trajectory for
         * @param pathIn [in] The path for follow
         * @param betaIn [in] Blend distances
         * @param vscaleIn [in] The velocity scale [0;1]
         * @param ascaleIn [in] The acceleration scale [0;1]
         * @param verbose [in] True to print our debug text
         */
		BlendedTrajectory(rw::models::Device::Ptr deviceIn,
		                  const rw::trajectory::QPath& pathIn,
		                  const std::vector<double>& betaIn,
		                  const double vscaleIn,
		                  const double ascaleIn,
		                  const bool verbose = false);

        /**
         * @brief Destructor
         */
        virtual ~BlendedTrajectory();


        //! @copydoc Trajectory::x(double) const
        T x(double t) const;

        //! @copydoc Trajectory::dx(double) const
        T dx(double t) const;

        //! @copydoc Trajectory::ddx(double) const
        T ddx(double t) const;

        //! @copydoc Trajectory::duration(double) const
        double duration() const { return endTime(); }

        //! @copydoc Trajectory::startTime(double) const
        double startTime() const { return 0.0; }

        //! @copydoc Trajectory::endTime(double) const
        double endTime() const { return t_total; }

		//! @copydoc Trajectory<T>::getIterator(double) const
		virtual typename TrajectoryIterator<T>::Ptr getIterator(double dt) const {
			return rw::common::ownedPtr(new BlendedTrajectoryIterator<T>(const_cast<BlendedTrajectory*>(this), dt));
		}
		

    private:
        // Algorithm functions
        bool init();
        bool checkPath();
        void updateLimits();
        void updateWI();
        void updateWF();
        void updateWFnext();
        void updateWInext();
        void updatedwSMax();
        void updateddwSMax();
        void updateddwSMin();
        T findQext(T& qI, T& q, T& qF);
        double findDWI(T& deltaq, T& qI, T& qext, double wI, double wFprev, double dwFprev, double dwSMax, double ddwSMax, T& aBMax);
        double findDWF(T& deltaq, T& qI, T& qext, T& qextnext, T& qF, T& qInext, T& deltaqnext, T& deltaqnextnext, T& dqI, T& dqMax, T& aBMin, T& aBMax, T& aBMaxNext);
        double tau(double x, double uI, double uF, double a);
        double findDWSReached(double wI, double dwI, double wFprev, double dwFprev, double dwSMax, double ddwSMax, double ddwSMin);
        double findTSdecc(double dwI, double dwSreached, double ddwSMin);
        double findTS(double wI, double dwI, double wFprev, double dwFprev, double ddwSMax, double ddwSMin, double dwSreached, double TSdecc);
        double findTB(T& qI, T& qF, T& qext, T& dqI, T& dqF, T& aBMax, T& aBMin, T& deltaq, T& deltaqnext);

        static double phimin(double v, double a, double tau, double t);
        static double dphimin(double v, double a, double tau, double t);
        static double ddphimin(double v, double a, double tau, double t);
        static double phimax(double v, double a, double tau, double t);
        static double dphimax(double v, double a, double tau, double t);
        static double ddphimax(double v, double a, double tau, double t);
        static double phistarmin(double v0, double v, double a, double tau, double t);
        static double dphistarmin(double v0, double v, double a, double tau, double t);
        static double ddphistarmin(double v0, double v, double a, double tau, double t);
        static double phistarmax(double v0, double v, double a, double tau, double t);
        static double dphistarmax(double v0, double v, double a, double tau, double t);
        static double ddphistarmax(double v0, double v, double a, double tau, double t);

        // Class variables
        bool verbose;
		rw::models::Device::Ptr device; // Device
        unsigned int K; // DOF
        rw::trajectory::QPath path; // Path
        unsigned int Npath; // Path size
        std::vector<double> betas; // Blend parameters
        double beta, betaNext; // Current and next blend parameter
        double vscale, ascale; // Scale factors of maximum velocity and acceleration
        double t, t_total; // Current time and total trajectory time
        unsigned int confNum; // Current configuration index in path
        T jointBoundsMin, jointBoundsMax; // Joint bounds
        T aSMaxConst, aSMinConst, aSnextMinConst; // Linear segment acceleration limimts
        T aBMaxConst, aBMinConst; // Blend segment acceleration limits
        T dqMax; // Global velocity limit
        T aSMax, aSMin, aSnextMin; // Current linear segment acceleration limits
        T aBMax, aBMin, aBMaxNext; // Current blend segment acceleration limits
        double TI, TF, TS, TB, TSdecc, TFprev; // Time instances and intervals
        std::vector<double> Tmid, deltaTmid; // Blend midpoint
        T aI, aF, a, dqav, delta, dqIF, vmin, dqI, dqF; // Various velocities and accelerations
        T qFprev, qprev, q, qnext, qnextnext, qext, qextnext, deltaq, deltaqnext, deltaqnextnext, qI, qInext, qF, qFnext; // Configurations
        double wI, wInext, wFprev, wF, wFnext, dwI, dwFprev, dwF, dwSMax, ddwSMax, ddwSMin, dwSreached; // Weights
        bool eq2a, eq2b; // Equations
        // Linear segment functor
        struct funcLinear {
            double _TFprev, _TS, _TSdecc, _qFprevk, _dwFprev, _deltaqk, _dwSreached, _ddwSMax, _ddwSMin;
            funcLinear() {}
            funcLinear(double TFprev, double TS, double TSdecc, double qFprevk, double dwFprev,
                double deltaqk, double dwSreached, double ddwSMax, double ddwSMin) :
            _TFprev(TFprev), _TS(TS), _TSdecc(TSdecc), _qFprevk(qFprevk), _dwFprev(dwFprev),
                _deltaqk(deltaqk), _dwSreached(dwSreached), _ddwSMax(ddwSMax), _ddwSMin(ddwSMin) {}
            double x(double t) const {
                if(t <= _TFprev + _TS - _TSdecc ) {
                    return _qFprevk + phistarmin(_dwFprev*_deltaqk, _dwSreached*_deltaqk, _ddwSMax*_deltaqk, _TS-_TSdecc, t-_TFprev);
                } else {
                    return _qFprevk + phistarmin(_dwFprev*_deltaqk, _dwSreached*_deltaqk, _ddwSMax*_deltaqk, _TS-_TSdecc, _TS-_TSdecc) + _dwSreached * _deltaqk * (t - _TFprev - (_TS - _TSdecc)) + 0.5 * _ddwSMin * _deltaqk * (t - _TFprev - (_TS - _TSdecc)) * (t - _TFprev - (_TS - _TSdecc));
                }
            }
            double dx(double t) const {
                if(t <= _TFprev + _TS - _TSdecc ) {
                    return dphistarmin(_dwFprev*_deltaqk, _dwSreached*_deltaqk, _ddwSMax*_deltaqk, _TS-_TSdecc, t - _TFprev);
                } else {
                    return _dwSreached*_deltaqk+_ddwSMin*_deltaqk*(t - _TFprev - _TS + _TSdecc);
                }
            }
            double ddx(double t) const {
                if(t <= _TFprev + _TS - _TSdecc ) {
                    return ddphistarmin(_dwFprev*_deltaqk, _dwSreached*_deltaqk, _ddwSMax*_deltaqk, _TS-_TSdecc, t - _TFprev);
                } else {
                    return _ddwSMin * _deltaqk;
                }
            }
        };
        // First blend part functor
        enum BlendType {TURNING1, TURNING2, NONTURNING1, NONTURNING2, NONTURNING3};
        struct funcIext {
            //double _qextk, _dqIk, _aIk, _deltaTmidk, _Tmidk, _dqIFk, _aBMink, _aBMaxk, _vmink;
            double _aIk, _dqIFk, _aBMink, _aBMaxk, _vmink, _qextk, _dqIk, _deltaTmidk, _Tmidk;
            BlendType _type;
            funcIext() {}
            funcIext(BlendType type , double qextk, double dqIk, double deltaTmidk, double Tmidk,
                double aIk = 0.0, double dqIFk = 0.0, double aBMink = 0.0, double aBMaxk = 0.0, double vmink = 0.0) :
            _aIk(aIk), _dqIFk(dqIFk), _aBMink(aBMink), _aBMaxk(aBMaxk), _vmink(vmink), 
                _qextk(qextk), _dqIk(dqIk), _deltaTmidk(deltaTmidk), _Tmidk(Tmidk), _type(type) {}
            double x(double t) const {
                switch(_type) {
        case TURNING1:
            return _qextk - phimax(_dqIk, _aIk, _deltaTmidk, _Tmidk - t);
        case TURNING2:
            return _qextk - phimin(_dqIk, -_aIk, _deltaTmidk, _Tmidk - t);
        case NONTURNING1:
            return _qextk - phistarmin(_dqIFk, _dqIk, -_aBMink, _deltaTmidk, _Tmidk - t);
        case NONTURNING2:
            return _qextk - phistarmin(_dqIFk, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
        case NONTURNING3:
            return _qextk - phistarmax(_vmink, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
                }
                return 0.0;
            }
            double dx(double t) const {
                switch(_type) {
        case TURNING1:
            return dphimax(_dqIk, _aIk, _deltaTmidk, _Tmidk - t);
        case TURNING2:
            return dphimin(_dqIk, -_aIk, _deltaTmidk, _Tmidk - t);
        case NONTURNING1:
            return dphistarmin(_dqIFk, _dqIk, -_aBMink, _deltaTmidk, _Tmidk - t);
        case NONTURNING2:
            return dphistarmin(_dqIFk, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
        case NONTURNING3:
            return dphistarmax(_vmink, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
                }
                return 0.0;
            }
            double ddx(double t) const {
                switch(_type) {
        case TURNING1:
            return -ddphimax(_dqIk, _aIk, _deltaTmidk, _Tmidk - t);
        case TURNING2:
            return -ddphimin(_dqIk, -_aIk, _deltaTmidk, _Tmidk - t);
        case NONTURNING1:
            return -ddphistarmin(_dqIFk, _dqIk, -_aBMink, _deltaTmidk, _Tmidk - t);
        case NONTURNING2:
            return -ddphistarmin(_dqIFk, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
        case NONTURNING3:
            return -ddphistarmax(_vmink, _dqIk, -_aBMaxk, _deltaTmidk, _Tmidk - t);
                }
                return 0.0;
            }
        };
        // Last blend part functor
        struct funcExtF {
            //double _qextk, _dqFk, _aFk, _TB, _deltaTmidk, _Tmidk, _dqIFk, _aBMink, _aBMaxk, _vmink;
            double _aFk, _dqIFk, _aBMink, _aBMaxk, _vmink, _qextk, _dqFk, _TB, _deltaTmidk, _Tmidk;
            BlendType _type;
            funcExtF() {}
            funcExtF(BlendType type , double qextk, double dqFk, double TB, double deltaTmidk, double Tmidk,
                double aFk = 0.0, double dqIFk = 0.0, double aBMink = 0.0, double aBMaxk = 0.0, double vmink = 0.0) :
            _aFk(aFk), _dqIFk(dqIFk), _aBMink(aBMink), _aBMaxk(aBMaxk), _vmink(vmink),
                _qextk(qextk), _dqFk(dqFk), _TB(TB), _deltaTmidk(deltaTmidk), _Tmidk(Tmidk), _type(type) {}
            double x(double t) const {
                switch(_type) {
        case TURNING1:
            return _qextk + phimax(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case TURNING2:
            return _qextk + phimin(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING1:
            return _qextk + phistarmin(_dqIFk, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING2:
            return _qextk + phistarmin(_dqIFk, _dqFk, _aBMaxk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING3:
            return _qextk + phistarmax(_vmink, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
                }
                return 0.0;
            }
            double dx(double t) const {
                switch(_type) {
        case TURNING1:
            return dphimax(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case TURNING2:
            return dphimin(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING1:
            return dphistarmin(_dqIFk, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING2:
            return dphistarmin(_dqIFk, _dqFk, _aBMaxk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING3:
            return dphistarmax(_vmink, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
                }
                return 0.0;
            }
            double ddx(double t) const {
                switch(_type) {
        case TURNING1:
            return ddphimax(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case TURNING2:
            return ddphimin(_dqFk, _aFk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING1:
            return ddphistarmin(_dqIFk, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING2:
            return ddphistarmin(_dqIFk, _dqFk, _aBMaxk, _TB - _deltaTmidk, t - _Tmidk);
        case NONTURNING3:
            return ddphistarmax(_vmink, _dqFk, _aBMink, _TB - _deltaTmidk, t - _Tmidk);
                }
                return 0.0;
            }
        };
        // Trajectory lists
        std::vector<std::vector<funcLinear> > sList;
        std::vector<std::vector<funcIext> > iExtList;
        std::vector<std::vector<funcExtF> > extFList;
        std::vector<double> TIList, TFList;
        std::vector<std::vector<double> > TmidList;

        public:

            const std::vector<double>& getBlendStartTimes() {
                return TIList;
            }

            const std::vector<double>& getBlendEndTimes() {
                return TFList;
            }

		private:
			
		/**
		 * @brief Bi-directional iterator for running efficiently through a trajectory
		 */
		template <class U>
		class BlendedTrajectoryIterator: public TrajectoryIterator<U>
		{
		public:
			/**
			 * @brief Constructs iterator for \b trajectory
			 *
			 * @param trajectory [in] Trajectory to iterate through
			 * @param dt [in] Default stepsize used for ++ and -- operators
			 */
			BlendedTrajectoryIterator(typename BlendedTrajectory<U>::Ptr trajectory, double dt = 1)
			{
				_trajectory = trajectory;
				_dt = dt;
				_time = _trajectory->startTime();
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
			bool isEnd() const { return _time >= _trajectory->duration(); }

			/**
			 * @copydoc TrajectoryIterator::isBegin()
			 */
			bool isBegin() const { return _time <= 0; }

			/**
			 * @copydoc TrajectoryIterator::operator*()
			 */
			U operator*() const { return x(); }

			/**
			 * @copydoc TrajectoryIterator::x()
			 */
			U x() const {
				return _trajectory->x(_time);
			}

			/**
			 * @copydoc TrajectoryIterator::dx()
			 */
			U dx() const {
				return _trajectory->dx(_time);
			}

			/**
			 * @copydoc TrajectoryIterator::ddx()
			 */
			U ddx() const {
				return _trajectory->ddx(_time);
			}

		private:
			typename BlendedTrajectory<U>::Ptr _trajectory;
			double _time;
			double _dt;
		};

    }; // End class

}} // End namespaces

#endif
