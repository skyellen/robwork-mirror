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

#ifndef RWHW_MOTOMANIA20_HPP
#define RWHW_MOTOMANIA20_HPP

/**
 * @file MotomanIA10.hpp
 */

#include <rwhw/serialport/SerialPort.hpp>
#include <rw/common/macros.hpp>
#include <rw/common/TimerUtil.hpp>
#include <rw/math/Q.hpp>

namespace rwhw {
    /** @addtogroup MotomanIA20 */
    /*@{*/

/**
 * @brief Provides simple communication with a Motoman IA20
 *
 * Connects to a MotomanIA20 on a serial port and sends joints positions.
 * This communication only works if program listening to the serial port
 * is running on the robot.
 */
class MotomanIA20
{
public:
    /**
     * @brief Constructs MotomanIA20 and connects to the robot
     * @param port [in] Port to connect on
     */
    MotomanIA20(std::string port);

    /**
     * @brief Destructor. Closes connection to the robot
     */
	virtual ~MotomanIA20();

	/**
	 * @brief Sends target positions for the 7 joints
	 * @param q [in] Joint positions encoded as radians
	 */
	void setPosition(rw::math::Q);

	/**
	 * @brief Sends target position for the encoders of the 7 joints
	 * @param raw [in] Encoder targets
	 */
	void sendRaw(std::vector<int> raw);

private:
	rwhw::SerialPort* _serialPort;


};
/*@}*/


} //end namespace

#endif /*RWHW_MOTOMANIA20_HPP*/
