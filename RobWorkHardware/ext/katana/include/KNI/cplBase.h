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
#ifndef _CPLBASE_H_
#define _CPLBASE_H_
/****************************************************************************/
#include "cdlBase.h"
#include "dllexport.h"
/****************************************************************************/

#if !defined (BYTE_DECLARED)
#define BYTE_DECLARED
typedef unsigned char byte;	//!< type specification (8 bit)
#endif

//--------------------------------------------------------------------------//

/*!	\brief	Return value of 'comm()'
 */

typedef enum TRetCOMM {
	COMM_NO_ERR,		//!< communication was successful
	COMM_RTOERR,		//!< reading timeout occured
	COMM_CRCERR,		//!< CRC checking failed
	//COMM_SLVERR		//   slave error in the robot bus
};

//--------------------------------------------------------------------------//

/*!	\brief Abstract base class for protocol definiton
 *
 *	The robot can be controled by using different kind of protocols; this
 *	class has been introduced as an abstract base class to manage them
 *	gether; every protocol the robot should use in futur shoud be derived
 *	from this class.
 */

class DLLDIR CCplBase {

protected:
	CCdlBase*	device;	//!< communication device

public:

	/*!	\brief	Basic initializing function
	 *
	 *	The children of this class should write their initializing part in
	 *	that function.
	 */
	virtual bool init(CCdlBase* _device, byte _kataddr = 24) = 0;

	/*!	\brief	Base communication function

	 *
	 *	The children of this class should write their main double way
	 *	communication in this function.
	 */
	virtual TRetCOMM comm(const byte* _pack, byte* _buf, byte* _size) = 0;
	/*! \brief destructor
	 *
	 * This class is only an interface
	 */
	virtual ~CCplBase() {};
};

/****************************************************************************/
#endif //_CPLBASE_H_
/****************************************************************************/
