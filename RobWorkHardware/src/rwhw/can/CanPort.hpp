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

#ifndef RWHW_CANPORT_HPP
#define RWHW_CANPORT_HPP

#include <vector>
#include <string>

namespace rwhw {

    /** @addtogroup can */
    /*@{*/

    /**
     * @brief CanPort provides an interface for communication through CAN bus.
     *
     */

    class CanPort
    {

    public:

        /**
         * @brief The structure for transmitting and recieving can messg  es
         */
        struct CanMessage {
            unsigned int timeStamp;  // Timestamp for receive queue objects
            unsigned int id;         // Identifier 11/29-Bit
            unsigned char length;    // number of data bytes ( 0~8)
            unsigned char rtr;       // RTR-Bit: 0=dataframe, 1=Remoteframe
            unsigned char data[8];   // Array for up to 8 data bytes
        };


    public:

        /**
         * @brief Destructor
         */
        virtual ~CanPort() {};

        /**
         * @brief returns wether this can port is open.
         */
        virtual bool isOpen() = 0;

        /**
         * @brief opens this can port
         */
        virtual bool open(/* baudrate, 11/29bit option,  */) = 0;

        virtual bool open(int idlow, int idhigh) = 0;

        /**
         * @brief closes this can port
         */
        virtual void close() = 0;

        /**
         * @brief reads a message from the recieve buffer
         */
        virtual bool read( CanPort::CanMessage  &msg) = 0;

        /**
         * @brief writes message to the transmit buffer
         */
        virtual bool write(
            unsigned int id,
            const std::vector<unsigned char>& data) = 0;
    };

    /*@}*/

} // namespace rwhw


#endif /*RWHW_CANPORT_HPP */
