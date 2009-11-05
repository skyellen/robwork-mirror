/*
 * CubePort.hpp
 *
 *  Created on: 21-10-2008
 *      Author: jimali
 */

#ifndef CUBEPORT_HPP_
#define CUBEPORT_HPP_

#include <rwhw/serialport/SerialPort.hpp>
#include <rwhw/can/CanPort.hpp>

class Cmd;

/**
 * @brief the this class wraps the communication to external Cubes.
 * Since the cube communicates over RS232, CAN and Profibus this class
 * will enable reuse of the Cube class and make sure the commuication
 * specific stuff is hiden.
 */
class CubePort {
public:

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


#endif /* CUBEPORT_HPP_ */
