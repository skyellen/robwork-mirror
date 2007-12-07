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
#ifndef _CDLCOM_H_
#define _CDLCOM_H_
/****************************************************************************/
#include "cdlBase.h"
#include "dllexport.h"
/****************************************************************************/
//#define WIN32		//--> for WIN32
//#define LINUX		//--> for LINUX (default)
/****************************************************************************/

//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//
	#include <windows.h>
//-------------------------------------------------------//
#else //LINUX
//-------------------------------------------------------//
	#include <termios.h>
	#include <fcntl.h>
	#include <unistd.h>	

        //#include <iostream>
	#include <string>
//-------------------------------------------------------//
#endif //WIN32 else LINUX
//-------------------------------------------------------//

/*! \brief	This structrue stores the attributes for a
 *			serial port device.
 */

struct TCdlCOMDesc {
	int	port;	//!<	serial port number
	int	baud;	//!<	baud rate of port
	int	data;	//!<	data bit
	int	parity;	//!<	parity bit
	int	stop;	//!<	stop bit
	int	rttc;	//!<	read  total timeout
	int	wttc;	//!<	write total timeout
};

//--------------------------------------------------------------------------//


/*!	\brief	Encapsulates the serial port device.
 *
 *	This class is responsible for direct communication with the serial port
 *	device. It builds the lowest layer for communication and uses the system
 *	API functions to get access the to the device.
 */

class DLLDIR CCdlCOM : public CCdlBase {

protected:

	TCdlCOMDesc	ccd;	//!< Stores the attributes of the serial port device.

//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//
	HANDLE			prtHdl;	//!< port handle
	COMMTIMEOUTS	oto;	//!< old timeouts
//-------------------------------------------------------//
#else //LINUX
//-------------------------------------------------------//
	int				prtHdl;	//!< port handle
	struct termios	oto;	//!< old timeouts
//-------------------------------------------------------//
#endif //WIN32 else LINUX
//-------------------------------------------------------//

protected:

	/*! \brief	Converts an integer to a char.
	 */
	static char digit(const int _val) {
		return (char)((int)'0' + _val);
	}

public:
	/*! \brief	Construct a CCdlCOM class
	 *
	 *	To this constructor a 'TCdlCOMDesc' parameter has to be given, which
	 *	describes the desired serial port. An attempt to open a connection
	 *	to the desired device will be tried and if successful, 'lastOP()'
	 *	will return 'lopDONE', otherwise 'lopFAIL'.
	 */
	CCdlCOM(TCdlCOMDesc _ccd);

	/*! \brief	Destructs the class
	 */	virtual ~CCdlCOM();

	/*!	\brief	Sends data to the device
	 */	virtual int  send(const void* _buf, int _size);

	/*! \brief	Receives data from the device
	*/	virtual int  recv(void* _buf, int _size);
};

/****************************************************************************/
#endif //_CDLCOM_H_
/****************************************************************************/
