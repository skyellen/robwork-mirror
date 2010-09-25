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

#ifndef RWHW_SERIALPORT_HPP
#define RWHW_SERIALPORT_HPP

/**
 * @file SerialPort.hpp
 */
#if (defined _WIN32) || (defined __CYGWIN__)
#include <windows.h>
#endif

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <iostream>
#include <cstring>

namespace rwhw {

    /** @addtogroup serialport */
    /*@{*/

    /**
     * @brief SerialPort provides an interface for communication with a serial port
     *
     * TODO: It should be specfied whether blocking or non-blocking functions are
     * used. Or it should all be supported.
     */
    class SerialPort {
    public:
        /**
         * @brief Baudrate for communication
         *
         */
        enum Baudrate {
            Baud1200,
            Baud2400,
            Baud4800,
            Baud9600,
            Baud19200,
            Baud38400,
            Baud57600,
            Baud115200,
            Baud230400,
            Baud460800,
            Baud921600
        };

        /**
         * @brief stopbits for communication
         */
        enum StopBits { Stop1_0, Stop1_5, Stop2_0 };

        /**
         * @brief Parity for communication
         */
        enum Parity { Even, Odd, None, Mark, Space};

        /**
         * @brief DataBits for communication
         */
        enum DataBits {Data5, Data6, Data7, Data8};

    public:
        /**
         * @brief Baudrate setup parameters
         */
        SerialPort();

        /**
         * @brief Virtual destructor
         */
        virtual ~SerialPort();

        /**
         * @brief Open a serial port connection
         * @param port [in] port string identifier.
         * @param baudrate [in] selected baudrate
         * @param dbits [in] databits
         * @param parity [in] the parity
         * @param stopbits [in] stopbits
         * @return true is the connection was successfully established
         */
        virtual bool open(
            const std::string& port,
            Baudrate baudrate,
            DataBits dbits = Data8,
            Parity parity = None,
            StopBits stopbits = Stop1_0);

        /**
         * @brief Closes serial port connection
         */
        virtual void close();

        /**
         * @brief Writes characters to port. The operation is non blocking.
         * @param buf [in] buffer containing characters to send
         * @param n [in] number of characters
         */
        virtual bool write(const char* buf, int n);

        /**
         * @brief Reads characters from port. The read operation is non blocking
         * and returns the number of bytes that was actually read.
         * @param buf [out] buffer into which characters should be stored
         * @param n [in] number of characters to read
         * @return number of characters that was actually read
         *
         */
        virtual int read(char* buf, int n);

        /**
         * @brief Reads n characters from the serial port using calls to
         * the non blocking read function. This operation continues reading
         * until n bytes are read or timeout.
         */
        virtual bool read(
            char* buf,
            const unsigned int n,
            const unsigned int timeout,
            const unsigned int sInterval);

        /**
         * @brief Reads all characters from the serial port buffer
         */
        virtual void clean();

    private:
        SerialPort(const SerialPort&);
        SerialPort& operator=(const SerialPort&);
		#if (defined _WIN32) || (defined __CYGWIN__)
		HANDLE cfd;
		#endif
    };

    typedef rw::common::Ptr<SerialPort> SerialPortPtr;

    /**@}*/
} // end namespace

#endif // end include guard
