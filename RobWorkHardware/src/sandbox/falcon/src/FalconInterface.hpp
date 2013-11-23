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

#ifndef RWHW_FALCONINTERFACE_HPP
#define RWHW_FALCONINTERFACE_HPP

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <falcon/core/FalconDevice.h>



namespace rwhw {
	
/**
 * @class FalconInterface
 * @brief Provides basic interface to Novint Falcon 3D controller.
 */
class FalconInterface
{
	public: // typedefs
		typedef rw::common::Ptr<FalconInterface> Ptr;
		
	public: // constants
		/// Button states for the spherical grip
		enum Buttons { ButtonUp = 0x02, ButtonDown = 0x04, ButtonLeft = 0x08, ButtonRight = 0x01 };
		
		static const double zAxisOffset = -0.115;
		static const double gravityForce = 0.5; // arbitrary?
	
	public: // constructors
		/// Default constructor
		FalconInterface();
		
		/// Destructor
		virtual ~FalconInterface();
		
	public: // methods
		/// Starts Falcon I/O loop
		virtual void start();
		
		/// Stops Falcon I/O loop
		virtual void stop();
		
		/// Returns a bool indicating if Falcon loop is running
		virtual bool isRunning() const { return _running; }
		
		/// Returns a vector indicating current position
		virtual rw::math::Vector3D<> getPosition() const;
		
		/// Returns a raw state of the buttons on the grip
		virtual unsigned int getButtonState() const;
		
		/// Sets force to act on the grip
		virtual void setForce(const rw::math::Vector3D<>& force);
		
		/// Gets the current force acting on the grip
		virtual rw::math::Vector3D<> getForce() const;
		
		/**
		 * @brief Sets centering mode
		 * 
		 * Tries to keep the Falcon handle in the middle of its space
		 * by applying force proportional to handle position.
		 * It is possible to specify the force coefficient, as well as
		 * the dead zone.
		 * When centering mode is enabled, the force set by setForce() is disabled.
		 * 
		 * @param enable [in] set @a true to enable centering mode
		 * @param forceCoeff [in] proportionality coefficient for the centering force
		 * @param deadRadius [in] radius from the center for which the centering does not apply
		 */
		virtual void setCenteringMode(bool enable, double forceCoeff=100.0, double deadRadius=0.0);
		
		/**
		 * @brief Sets stick-to-axes mode
		 * 
		 * When moving the handle out of the resting position, it tries to make it stick to
		 * device axes.
		 * It can be enabled together with centering mode.
		 * 
		 * @param enable [in] set @a true to enable sticky mode
		 * @param forceCoeff [in] proportionality coefficient for the stickying force
		 * @param stickRadius [in] radius from the axis where the stickying force applies
		 */
		virtual void setStickyMode(bool enable, double forceMax=2.0, double stickyRadius=0.0075);
		
	protected: // methods
		/**
		 * @brief Initializes the Falcon interface
		 * 
		 * This should be always called before attempting to use device.
		 * Searches and connects with the device.
		 */
		virtual bool initialize();
		
		/// Falcon thread loop
		virtual void falconLoop();
		
		
		
	protected: // data
		libnifalcon::FalconDevice* _falcon; // libnifalcon device class
		
		bool _running; // indicates if the falcon thread is running
		boost::thread* _thread;
		mutable boost::mutex _mutex;
		
		boost::array<double, 3> _position; // current Falcon position
		unsigned int _buttonState; // state of the grip buttons
		boost::array<double, 3> _force; // current Falcon force
		
		rw::math::Vector3D<> _centerPosition;
		
		bool _centeringMode;
		double _centeringForceCoeff;
		double _centeringDeadZoneRadius;
		
		bool _stickyMode;
		double _stickyForceCoeff;
		double _stickyRadius;
};

} // end namespace

#endif /*RWHW_FALCONINTERFACE_HPP*/
