/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_io_serialport_SerialPort_HPP
#define rwlibs_io_serialport_SerialPort_HPP

/**
 * @file SerialPort.hpp
 */
#include <rw/common/TimerUtil.hpp>
#include <iostream>
#include <string>

namespace rwlibs { namespace io {

    /** @addtogroup io */
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
        virtual bool read(char* buf, 
                          const unsigned int n, 
                          const unsigned int timeout, 
                          const unsigned int sInterval)
        {
            unsigned int index = 0;
            const unsigned long time = rw::common::TimerUtil::CurrentTimeMs()+timeout;
            unsigned long currTime = time;
            do {
                index += read( &(buf[index]), n-index );
                if(index >= n){
                    return true;
                }
                rw::common::TimerUtil::SleepMs(sInterval);
                currTime = rw::common::TimerUtil::CurrentTimeMs();
            } while( currTime < time );
            return false;
        }
        
        /**
         * @brief Reads all characters from the serial port buffer
         */
        virtual void clean();

    private:
        SerialPort(const SerialPort&);
        SerialPort& operator=(const SerialPort&);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
