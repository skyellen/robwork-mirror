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

#include "CubePort.hpp"

#include <rw/common/macros.hpp>

#include "PCubeProtocol.hpp"

using namespace rwhw;
using namespace rw::common;

namespace {

    class CANCubePort : public CubePort {
    public:
        CANCubePort(rwhw::CanPort* port):_port(port){

        }

        virtual bool read(Message& msg){
            CanPort::CanMessage canMsg;
            if( _port->read(canMsg) ){
                msg.length = canMsg.length;
                msg.data[0] = canMsg.data[0];
                msg.data[1] = canMsg.data[1];
                msg.data[2] = canMsg.data[2];
                msg.data[3] = canMsg.data[3];
                msg.data[4] = canMsg.data[4];
                msg.data[5] = canMsg.data[5];
                msg.data[6] = canMsg.data[6];
                msg.data[7] = canMsg.data[7];
                msg.moduleAddr = canMsg.id & ~PCUBE_CANID_CMDACK;
                msg.rtr = canMsg.rtr;
                msg.timeStamp = canMsg.timeStamp;
            }
            return false;
        }

        virtual bool write(const Cmd& cmd, int moduleAddr){
            switch(cmd.cmdType){
            case(Cmd::ACK):
                return _port->write(PCUBE_CANID_CMDACK + moduleAddr, cmd.data);
            case(Cmd::GET):
                return _port->write(PCUBE_CANID_CMDGET + moduleAddr, cmd.data);
            case(Cmd::PUT):
                return _port->write(PCUBE_CANID_CMDPUT + moduleAddr, cmd.data);
            case(Cmd::ALL):
                return broadcast(cmd);
            default:
                RW_THROW("Unknown command type!");
            }
            return false;
        }

        virtual bool broadcast(const Cmd& cmd){
            return _port->write(PCUBE_CANID_CMDALL, cmd.data);
        }

    private:
        rwhw::CanPort *_port;
    };

    class SerialCubePort : public CubePort {
    public:
        SerialCubePort(SerialPort* port):_port(port)
        {

        }

        virtual bool read(Message& msg){
        	const unsigned int timeout = 200;
        	const unsigned int buffSize = 30;
        	unsigned char buff[buffSize];
			// READ STX and TELID with blocking read func
			if( !_port->read((char*)buff, 3, timeout, 2) ){
				return false;
			}
			if( buff[0] != 0x02)
				return false;
			// READ TELID
			msg.moduleAddr = ((buff[1]&0x03)<<3) | ((buff[2]>>5)&0x07);
			msg.length = buff[2]&0x0F;

        	if( buff[1]&0x08 ) msg.msgType = Message::ACK;
				else msg.msgType = Message::PUT;

			int n=0;
			unsigned int i=0;
			while(n<msg.length) {
				if( !_port->read((char*)buff+i, 1, timeout, 2 ) ){
					return false;
				}
				if( buff[i] == 0x10 ){
					++i;
					if( !_port->read((char*)buff+i, 1, timeout, 2 ) ){
						return false;
					}
					if( buff[i]==0x82 ){
						msg.data[n] = 0x02;
						++n;
					} else if( buff[i]==0x83 ){
						msg.data[n] = 0x03;
						++n;
					} else if (buff[i]==0x90 ){
						msg.data[n] = 0x10;
						++n;
					} else {
						return false;
					}
				} else {
					msg.data[n] = buff[i];
					++n;
				}
				++i;
				if(i>buffSize)
					return false;
			}
			if( !_port->read((char*)buff+i, 2, timeout, 2 ) ){
				return false;
			}
			// test BCC
			//if( buff[msg.length]!= 0x11 )
			//    return false;
			// test ETX
			if( buff[i+1]!= 0x03 )
				return false;
			return true;
		}

        virtual bool write(const Cmd& cmd, int moduleAddr){
            unsigned char buff[30];
            int n=0;
            int bcc = 0;
            buff[n++] = 0x02; // STX
            buff[n++] = 0x04 | ((moduleAddr>>3)&0x3); // TELID_SENDDAT
            bcc += buff[n-1];
            buff[n++] = ((moduleAddr&0x7)<<5) | cmd.data.size(); // TELID_SENDDAT
            bcc += buff[n-1];
            for(size_t i=0;i<cmd.data.size(); i++){
                if( cmd.data[i] == 0x02 ){
                    buff[n++] = 0x10;
                    buff[n++] = 0x82;
                } else if( cmd.data[i] == 0x03 ){
                    buff[n++] = 0x10;
                    buff[n++] = 0x83;
                } else if( cmd.data[i] == 0x10 ){
                    buff[n++] = 0x10;
                    buff[n++] = 0x90;
                } else {
                    buff[n++] = cmd.data[i];
                }
            }

            if( (int)cmd.data.size()!=n-3) {
                // Correct lower part of TELID
                buff[2] = ((moduleAddr&0x7)<<5) | (n-3); // TELID_SENDDAT
            }
            //int bcc = ((buff[1]<<8) | buff[2]) +  ;
            buff[n++] = 0x11; // BCC
            buff[n++] = 0x03; // ETX
            return _port->write((char*)buff, n);
        }

        virtual bool broadcast(const Cmd& cmd){
            RW_THROW("Broadcast not supported by serial connection!");
        }

    private:
        rwhw::SerialPort *_port;
    };

}


CubePort* CubePort::make(rwhw::SerialPort *port){
    return new SerialCubePort(port);
}

CubePort* CubePort::make(rwhw::CanPort *port){
    return new CANCubePort(port);
}

CubePort::~CubePort() {
  /* Empty */
}
