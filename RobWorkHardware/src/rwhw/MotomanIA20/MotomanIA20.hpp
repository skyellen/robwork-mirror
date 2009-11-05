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

} //end namespace

#endif /*RWHW_MOTOMANIA20_HPP*/
