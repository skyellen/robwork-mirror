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

#ifndef RW_TRAJECTORY_DEVICETRAJECTORY_HPP
#define RW_TRAJECTORY_DEVICETRAJECTORY_HPP

// RW
#include "Trajectory.hpp"
#include "Path.hpp"

namespace rw { namespace models { class Device; } }

namespace rw { 
namespace trajectory {
    
    /**
     * @brief Implements a trajectory with blends between segments.
     * TODO: Briefly describe how 
     *
     *
     */
    class DeviceTrajectory : public Trajectory<rw::math::Q> {
    public:

        /**
         * @brief Default constructor creating an empty trajectory
         */
        DeviceTrajectory(rw::common::Ptr<rw::models::Device> deviceIn, const rw::kinematics::State& state);

        /**
         * @brief Destructor
         */
        virtual ~DeviceTrajectory();


        //! @copydoc Trajectory::x(double) const
        rw::math::Q x(double t) const{ return _trajectory->x(t); }

        //! @copydoc Trajectory::dx(double) const
        rw::math::Q dx(double t) const{ return _trajectory->dx(t); }

        //! @copydoc Trajectory::ddx(double) const
        rw::math::Q ddx(double t) const{ return _trajectory->ddx(t); }

        //! @copydoc Trajectory::duration(double) const
        double duration() const { return _trajectory->duration(); }

        //! @copydoc Trajectory::startTime(double) const
        double startTime() const { return _trajectory->startTime(); }

        //! @copydoc Trajectory::endTime(double) const
        double endTime() const{ return _trajectory->endTime(); }

		//! @copydoc Trajectory<T>::getIterator(double) const
		virtual typename TrajectoryIterator<T>::Ptr getIterator(double dt) const {
			return rw::common::ownedPtr(new BlendedTrajectoryIterator<T>(const_cast<BlendedTrajectory*>(this), dt));
		}



		// use default SE3 blend from last configuration to target
		void moveL(const rw::math::Transform3D<>& target, double blend=0.0);

		// use default SE3 blend from last configuration to target
		void moveL(const rw::math::Rotation3D<>& target, double blend=0.0);

        // use default SE3 blend from last configuration to target
        void moveL(const rw::math::Vector3D<>& target, double blend=0.0);

		// use default blend from last configuration to target
		void moveJ(const rw::math::Q& target, double blend=0.0);



    private:
		rw::common::Ptr<rw::models::Device> _dev;
		InterpolatorTrajectory<rw::math::Q>::Ptr _trajectory;
        double _time, _timeTotal;
        double _dt;
        rw::kinematics::State _state;

    };

    }; // End class

}} // End namespaces

#endif
