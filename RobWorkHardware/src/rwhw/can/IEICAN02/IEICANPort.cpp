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

#include "IEICANPort.hpp"

#include <icanapi.h>

using namespace rwhw;

IEICANPort::IEICANPort(unsigned int cardIdx,unsigned int portNr):
	_cardIdx( cardIdx),
	_portNr( portNr ),
	_portOpen(false)
{

}

IEICANPort::~IEICANPort()
{
	if( _portOpen )
		close();
	// if( _instanceCnt == 0 )
	// 		closeIEILibrary()
}


IEICANPort* IEICANPort::getIEICANPortInstance(unsigned int cardIdx,unsigned  int portNr) // TODO: add baud ad can id type
{
	if( !isIEICAN02LibOpen() ){
		if( !openIEICAN02Library() ){
			std::cout << "Error loading IEIECAN02 dynamic library!!" << std::endl;
			return NULL;
		}
	}

	if( CAN_Init(cardIdx, portNr, BAUD_250K, CAN_11B) < 0){
		std::cout << "Error initializing can card!!" << std::endl;
		return NULL;
	}

	if( CAN_SetAccMask(cardIdx, portNr, 0, 0) < 0 ){
		std::cout << "Error setting AccMask!!" << std::endl;
		return NULL;
	}

	if( CAN_ConfigQueue(cardIdx, portNr, CAN_RX_QUE, 100) == 0){
		std::cout << "Error configuring RX queue!!" << std::endl;
		return NULL;
	}

	if( CAN_ConfigQueue(cardIdx, portNr, CAN_TX_QUE, 100) == 0){
		std::cout << "Error configuring RX queue!!" << std::endl;
		return NULL;
	}
	// TODO: add the instance to a static list so that we don't risk instantiating it twice
	return new IEICANPort(cardIdx,portNr);
}

bool IEICANPort::isOpen(){
	return _portOpen;
}

bool IEICANPort::open(/* baudrate, 11/29bit option,  */){
	if( CAN_Start(_cardIdx, _portNr ) > 0){
		_portOpen = true;
	} else {
		_portOpen = false;
	}
	return _portOpen;
}

void IEICANPort::close(){

	_portOpen = false;
	CAN_Reset( _cardIdx, _portNr );
}

bool IEICANPort::read( CanPort::CanMessage  &msg){
	if( CAN_CountMsgs( _cardIdx, _portNr, CAN_RX_QUE ) == 0 )
		return false;
	CAN_MSG canmsg;
	int res = CAN_ReadMsg(_cardIdx, _portNr, 1, &canmsg);
	if( res< 0) {
		std::cout << "Error read: error reading can port" << std::endl;
		return false;
	}
	msg.timeStamp = canmsg.time_stamp;
	msg.id = canmsg.id;
	msg.length = canmsg.len;
	msg.rtr = canmsg.rtr;
	for(int i=0; i<msg.length; i++){
		msg.data[i] =  canmsg.a_data[i];
	}
	return true;
}

bool IEICANPort::write(
    unsigned int id, const std::vector<unsigned char>& raw_data)
{
    // Take a copy so that we can provide constness of the input.
    std::vector<unsigned char> data(raw_data);

    unsigned char* msg = data.empty() ? NULL : &data.at(0);
    int res = CAN_SendMsg(_cardIdx, _portNr, id, data.size(), msg);

    if( res > 0) {
        return true;
    } else if ( res == 0 ){
        std::cout << "Error write: Queue is full" << std::endl;
    } else {
        std::cout << "Error write: CAN parameter error or CAN tx error" << std::endl;
    }

    return false;

/*	sleepCnt = 0;
	while( CAN_CountMsgs( _cardIdx, _portNr, CAN_RX_QUE ) == 0 ){
        // Todo: create timeout thing
        if( sleepCnt > 5 ){
        std::cout << "Write Error: timeout" << std::endl;
        return false;
        }
        sleepCnt++
        Sleep(2);
	}*/
}
