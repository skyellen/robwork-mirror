/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/******************************************************************************************************************/
#ifndef _CPLSERIAL_H_
#define _CPLSERIAL_H_
/******************************************************************************************************************/
#include "cplBase.h"
#include "dllexport.h"
/******************************************************************************************************************/


/*!	\brief	Header of a communication packet
 */
struct THeader {
	byte	size;		//!< header size
	byte	data[256];	//!< data part: 16x zero, 1x one, 1x katadr
};

/*!	\brief	Communication packet
 */
struct TPacket {
	byte	send_sz;	//!< send size of the packet
	byte	read_sz;	//!< read size of the packet
};

//----------------------------------------------------------------------------------------------------------------//


/*!	\brief	Base class of two different serial protocols
 */
class DLLDIR CCplSerial : public CCplBase {

protected:
	THeader	hdr;			//!< header
	TPacket cmd[256];		//!< command table

	byte	send_buf[256];	//!< sending buffer
	byte	read_buf[256];	//!< receive buffer

protected:
	virtual bool load_tbl()						= 0;	//!< Loads the command table from the robot's firmware.
	virtual void defineProtocol(byte _kataddr)	= 0;	//!< Defines the protocol's attributes.
};

//----------------------------------------------------------------------------------------------------------------//

/*! \brief	Implement the Serial-Zero protocol
 */
class DLLDIR CCplSerialZero : public CCplSerial {

protected:
	virtual bool load_tbl();					//!< Loads the command table from the robot's firmware.
	virtual void defineProtocol(byte _kataddr);	//!< Defines the protocol's attributes.

public:
	/*!	\brief	Initializing function
	 *
	 *	Init the protocols basic attributes.
	 */
	virtual bool init(CCdlBase* _device, byte _kataddr = 24);


	/*!	\brief	Communication function
	 *
	 *	Sends a communications packet and receives one from the robot.
	 */
	virtual TRetCOMM comm(const byte* _pack, byte* _buf, byte* _size);
};

//----------------------------------------------------------------------------------------------------------------//

/*! \brief	Implement the Serial-CRC protocol
 */
class DLLDIR CCplSerialCRC : public CCplSerial {

protected:
	virtual bool load_tbl();					//!< Loads the command table from the robot's firmware.
	virtual void defineProtocol(byte _kataddr);	//!< Defines the protocol's attributes.
	virtual TRetCOMM send(byte* _send_buf, byte _bufsz); // Sends a packet.
	virtual TRetCOMM recv(byte* _read_buf, byte read_sz, byte* _size); // Receives the packet and checks the CRC.

public:
	/*!	\brief	Initializing function
	 *
	 *	Init the protocols basic attributes.
	 */
	virtual bool init(CCdlBase* _device, byte _kataddr = 24);

	/*!	\brief	Communication function
	 *
	 *	Sends a communications packet and receives one from the robot.
	 */
	virtual TRetCOMM comm(const byte* _pack, byte* _buf, byte* _size);
};

/******************************************************************************************************************/
#endif //_CPLSERIALZERO_H_
/******************************************************************************************************************/
