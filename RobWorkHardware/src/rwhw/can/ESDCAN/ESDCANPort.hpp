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

#ifndef RWHW_ESDCANPORT_HPP
#define RWHW_ESDCANPORT_HPP

#include <rwhw/can/CanPort.hpp>

#include <vector>
#include <iostream>
#include <rw/common/Ptr.hpp>

#include <ntcan.h>
#if defined( OSNAME_LINUX ) && ! defined( NTCAN_HANDLE )
    // Linux ntcan.h uses HANDLE where Windows ntcan uses NTCAN_HANDLE:
    #define NTCAN_HANDLE HANDLE
#endif

namespace rwhw {

    /** @addtogroup can  */
    /*@{*/

    /**
     * @brief CanPort driver wrapper for the ESDCAN driver.
     */
    class ESDCANPort: public CanPort
    {
    public:
        typedef rw::common::Ptr<ESDCANPort> Ptr;
        /**
         * @brief Status struct
         */
        struct CanDeviceStatus {
            /** @brief The identifier/address of the device */
            int netid;
        };

        /**
         * @brief Baud settings for can bus
         */
        typedef enum {CanBaud1000=0, CanBaud666_6, CanBaud500, CanBaud333_3,
            CanBaud250, CanBaud166, CanBaud125, CanBaud100, CanBaud66_6,
            CanBaud50, CanBaud33_3, CanBaud20, CanBaud12_5, CanBaud10} CanBaud;

    private:
        /**
         *
         */
        ESDCANPort(unsigned int netId, long txQueueSize, long rxQueueSize, CanBaud canBaud, int transmitDelay);
        virtual ~ESDCANPort();

    public:

        /**
         * @brief Returns all devices connected to the can bus
         */
        static std::vector<CanDeviceStatus> getConnectedDevices();

        /**
         * Gets an instance of IEICANPort by specifiing card and port nr.
         */
        static ESDCANPort* getPortInstance(
            unsigned int netId, long txQueueSize=1, long rxQueueSize=1,
			CanBaud canBaud=CanBaud250, int transmitDelay=0); // TODO: add baud ad can id type

        /**
         * @copydoc CanPort::isOpen
         */
        bool isOpen();

        /**
         * @copydoc CanPort::open
         */
        bool open();
        bool open(int idlow, int idhigh);

        /**
         * @copydoc CanPort::close
         */
        void close();

        /**
         * @copydoc CanPort::read
         */
        bool read( CanPort::CanMessage  &msg);

        /**
         * @copydoc CanPort::write
         */
        bool write(unsigned int id, const std::vector<unsigned char>& data);

		void setBaudRate(CanBaud canBaud);
		unsigned int getNetId() {return _netId;}

		NTCAN_HANDLE getHandle() { return _handle; }

    private:
        unsigned int _netId;
        long _txQueueSize;
        long _rxQueueSize;
        CanBaud _canBaud;
        int _transmitDelay;
        bool _portOpen;
        NTCAN_HANDLE _handle;
    };

    /*@}*/

}

#endif /*ESDCANPORT_HPP_*/
