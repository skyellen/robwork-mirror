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

#ifndef CUBEPORT_HPP_
#define CUBEPORT_HPP_

#include <rwhw/serialport/SerialPort.hpp>
#include <rwhw/can/CanPort.hpp>


namespace rwhw {
    struct Cmd;

        /** @addtogroup PowerCube */
        /*@{*/

    /**
     * @brief the this class wraps the communication to external Cubes.
     * Since the cube communicates over RS232, CAN and Profibus this class
     * will enable reuse of the Cube class and make sure the commuication
     * specific stuff is hiden.
     */
    class CubePort {
    public:

      /* virtual destructor, making this class appropriate for inheritance */
      virtual ~CubePort();

        /**
         * @brief The structure for transmitting and recieving can messg  es
         */
        struct Message {
            typedef enum{GET,PUT,ACK,ALL} MsgType;
            unsigned int timeStamp;  // Timestamp for receive queue objects
            unsigned int moduleAddr; // Address of the Cube modedule
            MsgType msgType;
            unsigned char length;    // number of data bytes ( 0~8)
            unsigned char rtr;       // RTR-Bit: 0=dataframe, 1=Remoteframe
            unsigned char data[8];   // Array for up to 8 data bytes
        };


        static CubePort* make(rwhw::SerialPort *port);

        static CubePort* make(rwhw::CanPort *port);

        virtual bool read(Message& msg) = 0;

        virtual bool write(const Cmd& cmd, int moduleAddr) = 0;

        /**
         * @brief broadcast commands are only supported by the CAN
         * protocol. If used with serial protocol an exeption will be
         * thrown.
         * @param cmd
         * @return
         */
        virtual bool broadcast(const Cmd& cmd) = 0;

    };

    /*@}*/
} //namespace

#endif /* CUBEPORT_HPP_ */
