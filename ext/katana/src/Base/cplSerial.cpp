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


/****************************************************************************/
#include "KNI/cplSerial.h"
#include "KNI/cplSerialCRC.h"
#include "CRC.h"
#include <time.h>
//#include <memory>
#include <string>
/****************************************************************************/

bool CCplSerialZero::load_tbl() {

	// set table to zero -----------------------------------//

	memset(cmd,0,sizeof(cmd));

	// set command table from the katana -------------------//

	cmd[(int)'B'].send_sz	= 1;
	cmd[(int)'B'].read_sz	= 3;
	cmd[(int)'X'].send_sz	= 1;
	cmd[(int)'X'].read_sz	= 127;
	cmd[(int)'Y'].send_sz	= 1;
	cmd[(int)'Y'].read_sz	= 84;
	cmd[(int)'Z'].send_sz	= 1;
	cmd[(int)'Z'].read_sz	= 1;
	cmd[(int)'K'].send_sz	= 11;
	cmd[(int)'K'].read_sz	= 2;
	cmd[(int)'O'].send_sz	= 12;
	cmd[(int)'O'].read_sz	= 2;
	cmd[(int)'C'].send_sz	= 5;
	cmd[(int)'C'].read_sz	= 3;
	cmd[(int)'D'].send_sz	= 2;
	cmd[(int)'D'].read_sz	= 8;
	cmd[(int)'U'].send_sz	= 3;
	cmd[(int)'U'].read_sz	= 5;
	cmd[(int)'E'].send_sz	= 2;
	cmd[(int)'E'].read_sz	= 18;
	cmd[(int)'F'].send_sz	= 12;
	cmd[(int)'F'].read_sz	= 2;
	cmd[(int)'V'].send_sz	= 3;
	cmd[(int)'V'].read_sz	= 13;
	cmd[(int)'N'].send_sz	= 3;
	cmd[(int)'N'].read_sz	= 13;
	cmd[(int)'P'].send_sz	= 25;
	cmd[(int)'P'].read_sz	= 3;
	cmd[(int)'Q'].send_sz	= 7;
	cmd[(int)'Q'].read_sz	= 2;
	cmd[(int)'E'+128].send_sz	= 2;
	cmd[(int)'E'+128].read_sz	= 18;
	cmd[(int)'L'].send_sz	= 38;
	cmd[(int)'L'].read_sz	= 2;
	cmd[(int)'L'+128].send_sz	= 3;
	cmd[(int)'L'+128].read_sz	= 2;
	cmd[(int)'A'].send_sz	= 3;
	cmd[(int)'A'].read_sz	= 2;
	cmd[(int)'S'].send_sz	= 6;
	cmd[(int)'S'].read_sz	= 6;
	cmd[(int)'R'].send_sz	= 3;
	cmd[(int)'R'].read_sz	= 3;

	//------------------------------------------------------//

	return true;
}

void CCplSerialZero::defineProtocol(byte _kataddr) {
	hdr.size = 19;
	for (int i=0; i<16; i++) {	//fill header
		hdr.data[i] = 0;	//16x zero
	} hdr.data[16] = 1;		//convention
	hdr.data[17] = _kataddr;	//complete header
}

bool CCplSerialZero::init(CCdlBase* _device, byte _kataddr) {
	device = _device;
	defineProtocol(_kataddr);
	return load_tbl();
}

TRetCOMM CCplSerialZero::comm(const byte* _pack, byte* _buf, byte* _size) {
	//------------------------------------------------------------------//

	memset(send_buf,0,256);				//override old values
	hdr.data[hdr.size-1] = cmd[_pack[0]].send_sz;	//complete header
	memcpy(send_buf, hdr.data, hdr.size);
	memcpy(send_buf+hdr.size,_pack,hdr.data[hdr.size-1]);
	device->send(send_buf,hdr.size+hdr.data[hdr.size-1]);	//send to dev
	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;			//comm failed
	}

	//------------------------------------------------------------------//

	memset(read_buf,0,256);				//read through device
	*_size = device->recv(read_buf,cmd[_pack[0]].read_sz);
	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;			//comm failed
	}

	memcpy(_buf,read_buf,*_size);			//read_buf to _buf
	return COMM_NO_ERR;				//yeah :)

	//------------------------------------------------------------------//
}

/****************************************************************************/

bool CCplSerialCRC::load_tbl() {

	// set table to zero -----------------------------------//

	memset(cmd,0,sizeof(cmd));

	// set command table from the katana -------------------//

	cmd[(int)'B'].send_sz	= 1;
	cmd[(int)'B'].read_sz	= 3;
	cmd[(int)'X'].send_sz	= 1;
	cmd[(int)'X'].read_sz	= 127;
	cmd[(int)'Y'].send_sz	= 1;
	cmd[(int)'Y'].read_sz	= 84;
	cmd[(int)'Z'].send_sz	= 1;
	cmd[(int)'Z'].read_sz	= 1;
	cmd[(int)'K'].send_sz	= 11;
	cmd[(int)'K'].read_sz	= 2;
	cmd[(int)'O'].send_sz	= 12;
	cmd[(int)'O'].read_sz	= 2;
	cmd[(int)'C'].send_sz	= 5;
	cmd[(int)'C'].read_sz	= 3;
	cmd[(int)'D'].send_sz	= 2;
	cmd[(int)'D'].read_sz	= 8;
	cmd[(int)'U'].send_sz	= 3;
	cmd[(int)'U'].read_sz	= 5;
	cmd[(int)'E'].send_sz	= 2;
	cmd[(int)'E'].read_sz	= 18;
	cmd[(int)'F'].send_sz	= 12;
	cmd[(int)'F'].read_sz	= 2;
	cmd[(int)'V'].send_sz	= 3;
	cmd[(int)'V'].read_sz	= 13;
	cmd[(int)'N'].send_sz	= 3;
	cmd[(int)'N'].read_sz	= 13;
	cmd[(int)'P'].send_sz	= 25;
	cmd[(int)'P'].read_sz	= 3;
	cmd[(int)'Q'].send_sz	= 7;
	cmd[(int)'Q'].read_sz	= 2;
	cmd[(int)'E'+128].send_sz	= 2;
	cmd[(int)'E'+128].read_sz	= 18;
	cmd[(int)'L'].send_sz	= 38;
	cmd[(int)'L'].read_sz	= 2;
	cmd[(int)'L'+128].send_sz	= 3;
	cmd[(int)'L'+128].read_sz	= 2;
	cmd[(int)'A'].send_sz	= 3;
	cmd[(int)'A'].read_sz	= 2;
	cmd[(int)'S'].send_sz	= 6;
	cmd[(int)'S'].read_sz	= 6;
	cmd[(int)'R'].send_sz	= 3;
	cmd[(int)'R'].read_sz	= 3;

	//------------------------------------------------------//

	return true;
}

void CCplSerialCRC::defineProtocol(byte _kataddr) {
	hdr.size = 3;
	hdr.data[0] = 1;	//convention
	hdr.data[1] = _kataddr;	//complete header
}

bool CCplSerialCRC::init(CCdlBase* _device, byte _kataddr) {
	device = _device;
	defineProtocol(_kataddr);
	return load_tbl();
}

TRetCOMM CCplSerialCRC::comm(const byte* _pack, byte* _buf, byte* _size) {

	// This method enssamble the packet with the header, data, and CRC.
	// Sends it and receives the answer.

	memset(send_buf,0,256);							//override old values
	hdr.data[hdr.size-1] = cmd[_pack[0]].send_sz;	//complete header
	memcpy(send_buf, hdr.data, hdr.size);
	memcpy(send_buf+hdr.size,_pack,hdr.data[hdr.size-1]);

	short crc   = CRC_CHECKSUM((uint8*)_pack,hdr.data[hdr.size-1]);
	byte  bufsz = hdr.size + hdr.data[hdr.size-1];
	send_buf[bufsz++] = (byte)(crc >> 8);			//hi-byte
	send_buf[bufsz++] = (byte)(crc & 0xFF);			//lo-byte

	memset(read_buf,0,256);							//read through device
	byte read_sz = cmd[_pack[0]].read_sz + 2;

	//------------------------------------------------------------------//

	TRetCOMM send_return = COMM_RTOERR;
	TRetCOMM recv_return = COMM_RTOERR;
	short tries_send = 0;
	short tries_recv = 0;

	while (tries_send++ != NUMBER_OF_RETRIES_SEND && send_return != COMM_NO_ERR) {
		send_return = send(send_buf,bufsz);
	};
    if (send_return != COMM_NO_ERR) {
		return send_return;
	};

 	while (tries_recv++ != NUMBER_OF_RETRIES_RECV && recv_return != COMM_NO_ERR) {

			if ((recv_return = recv(read_buf,read_sz,_size)) == COMM_NO_ERR) {
				memcpy(_buf,read_buf,*_size);		// copy read_buf to _buf
				return recv_return;
			};

			tries_send = 0;
			send_return = COMM_RTOERR;

			clock_t t = clock();
			while (1000 * (clock() - t) < 25 * CLOCKS_PER_SEC) {} // wait 25 milliseconds

			while (tries_send++ != NUMBER_OF_RETRIES_SEND && send_return != COMM_NO_ERR) {
				send_return = send(send_buf,bufsz);
			};
			if (send_return != COMM_NO_ERR) {
				return send_return;
			};
			recv_return = COMM_RTOERR;

	};

	return recv_return;
}

TRetCOMM CCplSerialCRC::send(byte* _send_buf, byte _bufsz) {

	// Sends the packet.
        //printf("INFO: ");
        //for (int i=0; i<_bufsz; i++)
	//  printf("%d ",_send_buf[i]);
        //printf("\n");

	device->send(_send_buf,_bufsz);					//send to device
	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;							//comm failed
	}; return COMM_NO_ERR;
}

TRetCOMM CCplSerialCRC::recv(byte* _read_buf, byte read_sz, byte* _size) {

	// Receives the packet and checks the CRC.

	*_size = device->recv(_read_buf, read_sz);		//receives from device

	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;							//comm failed
	};

	*_size -= 2;

	byte bhi = read_buf[read_sz-2];
	byte blo = read_buf[read_sz-1];

	short crc = CRC_CHECKSUM((uint8*)read_buf,*_size);
	byte hi = (byte)(crc >> 8);
	byte lo = (byte)(crc & 0xFF);

	if ((hi != bhi) || (lo != blo)) {
		return COMM_CRCERR;							//CRC failed
	} return COMM_NO_ERR;

}

/****************************************************************************/
