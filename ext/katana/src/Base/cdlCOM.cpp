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
#include "KNI/cdlCOM.h"
#include <iostream>
/****************************************************************************/

//-------------------------------------------------------//
#ifdef WIN32
//-------------------------------------------------------//

CCdlCOM::CCdlCOM(TCdlCOMDesc _ccd) {
    //------------------------------------------------------------------//
    lop	= lopFAIL;
    ccd = _ccd;
    prtHdl	= INVALID_HANDLE_VALUE;
    //------------------------------------------------------------------//
    DCB commDCB;		//COM port parameters
    COMMTIMEOUTS nto;	//new timeouts
    char comX[5];
    char dcb[35];
    int i, d;
    //------------------------------------------------------------------//
    if (prtHdl != INVALID_HANDLE_VALUE) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    strncpy(comX, "COM ", 5);
    comX[3] = digit(ccd.port);
    prtHdl = CreateFile(comX,
			GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, 0
	);
    //------------------------------------------------------------------//
    if (prtHdl == INVALID_HANDLE_VALUE) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    FillMemory(&commDCB, sizeof(commDCB), 0);
    commDCB.DCBlength = sizeof(commDCB);
    strncpy(dcb, "baud=       parity=  data=  stop= ", 35);
    for(i=5,d=100000; d>=1; d=d/10) {
	if(d <= ccd.baud) { dcb[i++] = digit((ccd.baud/d) % 10); }
    }
    dcb[19] = ccd.parity;
    dcb[26] = digit(ccd.data);
    dcb[33] = digit(ccd.stop);
    if (!BuildCommDCB(dcb, &commDCB)) {
	CloseHandle(prtHdl);
	prtHdl = INVALID_HANDLE_VALUE;
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    commDCB.fAbortOnError	= false;
    commDCB.fInX			= false;
    commDCB.fOutX			= false;
    commDCB.fOutxCtsFlow	= false;
    if (!SetCommState(prtHdl, &commDCB)) {
	CloseHandle(prtHdl);
	prtHdl = INVALID_HANDLE_VALUE;
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    PurgeComm(	prtHdl,	PURGE_TXABORT | PURGE_RXABORT |
		PURGE_TXCLEAR | PURGE_RXCLEAR);
    //------------------------------------------------------------------//
    GetCommTimeouts(prtHdl, &oto);
    nto.ReadIntervalTimeout			= MAXDWORD;
    nto.ReadTotalTimeoutMultiplier	= 0;
    nto.ReadTotalTimeoutConstant	= ccd.rttc;
    nto.WriteTotalTimeoutMultiplier	= 0;
    nto.WriteTotalTimeoutConstant	= ccd.wttc;
    if (!SetCommTimeouts(prtHdl, &nto)) {
	CloseHandle(prtHdl);
	prtHdl = INVALID_HANDLE_VALUE;
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;
    //------------------------------------------------------------------//
}

CCdlCOM::~CCdlCOM() {
    //------------------------------------------------------------------//
    if(prtHdl == INVALID_HANDLE_VALUE) {
	lop = lopDONE;
	return;
    }
    //------------------------------------------------------------------//
    FlushFileBuffers(prtHdl);
    SetCommTimeouts(prtHdl, &oto);
    //------------------------------------------------------------------//
    if (!CloseHandle(prtHdl)) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    prtHdl = INVALID_HANDLE_VALUE;
    lop = lopDONE;
    //------------------------------------------------------------------//
}

int  CCdlCOM::send(const void* _buf, int _size) {
    //------------------------------------------------------------------//
    if (prtHdl == INVALID_HANDLE_VALUE) {
	lop = lopFAIL;
	return 0;
    } PurgeComm(prtHdl, PURGE_TXABORT | PURGE_TXCLEAR);
    //------------------------------------------------------------------//
    unsigned long readsz;
    if (!WriteFile(prtHdl, _buf, _size, &readsz, 0) || ((int)readsz != _size)) {
	lop = lopFAIL;
	return (int)readsz;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;
    return (int)readsz;
    //------------------------------------------------------------------//
}

int  CCdlCOM::recv(void* _buf, int _size) {
    //------------------------------------------------------------------//
    unsigned long readsz;
    if (prtHdl == INVALID_HANDLE_VALUE) {
	lop = lopFAIL;
	return 0;
    }
    //------------------------------------------------------------------//
    if (!ReadFile(prtHdl, _buf, _size, &readsz, 0) || ((unsigned)_size != readsz)) {
	lop = lopFAIL;
	return readsz;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;;
    PurgeComm(prtHdl, PURGE_RXABORT | PURGE_RXCLEAR);
    return (int)readsz;
    //------------------------------------------------------------------//
}


//-------------------------------------------------------//
#else //LINUX
//-------------------------------------------------------//

CCdlCOM::CCdlCOM(TCdlCOMDesc _ccd) {
    //------------------------------------------------------------------//
    lop	= lopFAIL;
    ccd	= _ccd;
    prtHdl	= -1;
    speed_t iospeed;
    //------------------------------------------------------------------//
    struct termios	nto;
    char		name[11];
    //------------------------------------------------------------------//
    if (prtHdl >= 0) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    strncpy(name, "/dev/ttyS ", 11);
    name[9] = digit(ccd.port);
    prtHdl = ::open(name, O_RDWR | O_NOCTTY |
		    O_NDELAY| O_NONBLOCK); // | O_SYNC);
    //------------------------------------------------------------------//
    if (prtHdl < 0) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    tcgetattr(prtHdl, &oto);
    bzero(&nto, sizeof(nto));
    nto.c_cc[VTIME]	= 0;
    nto.c_cc[VMIN]	= 0;
    nto.c_oflag	= 0;
    nto.c_lflag	= 0;
    nto.c_cflag	= CLOCAL | CREAD;
    nto.c_iflag	= 0;
    //------------------------------------------------------------------//
    switch (ccd.baud) {
	case     50:  iospeed = B50;     break;
	case     75:  iospeed = B75;     break;
	case    110:  iospeed = B110;    break;
	case    134:  iospeed = B134;    break;
	case    150:  iospeed = B150;    break;
	case    200:  iospeed = B200;    break;
	case    300:  iospeed = B300;    break;
	case    600:  iospeed = B600;    break;
	case   1200:  iospeed = B1200;   break;
	case   1800:  iospeed = B1800;   break;
	case   2400:  iospeed = B2400;   break;
	case   4800:  iospeed = B4800;   break;
	case   9600:  iospeed = B9600;   break;
	case  19200:  iospeed = B19200;  break;
	case  38400:  iospeed = B38400;  break;
	case  57600:  iospeed = B57600;  break;
	case 115200:  iospeed = B115200; break;
	case 230400:  iospeed = B230400; break;
    }
    //------------------------------------------------------------------//
    switch (ccd.data) {
	case  5:  nto.c_cflag |= CS5;  break;
	case  6:  nto.c_cflag |= CS6;  break;
	case  7:  nto.c_cflag |= CS7;  break;
	case  8:  nto.c_cflag |= CS8;  break;
    }
    //------------------------------------------------------------------//
    switch (ccd.parity) {
	case 'N':
	case 'n':					break;
	case 'O':
	case 'o':	nto.c_cflag |= PARENB | PARODD;	break;
	case 'E':
	case 'e':	nto.c_cflag |= PARENB;		break;
    }
    //------------------------------------------------------------------//
    switch (ccd.stop) {
	case  1:				break;
	case  2:	nto.c_cflag |= CSTOPB;	break;
    }
    //------------------------------------------------------------------//
    tcflush(prtHdl,TCIFLUSH);
    cfsetispeed(&nto,iospeed);
    cfsetospeed(&nto,iospeed);
    if (tcsetattr(prtHdl, TCSANOW, &nto) != 0) {
	::close(prtHdl);
	prtHdl = -1;
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;
    //------------------------------------------------------------------//
}

CCdlCOM::~CCdlCOM() {
    //------------------------------------------------------------------//
    if (prtHdl < 0) {
	lop = lopDONE;
	return;
    }
    //------------------------------------------------------------------//
    tcflush(prtHdl, TCIFLUSH);
    tcsetattr(prtHdl, TCSANOW, &oto);
    //------------------------------------------------------------------//
    if (::close(prtHdl) != 0) {
	lop = lopFAIL;
	return;
    }
    //------------------------------------------------------------------//
    prtHdl	= -1;
    lop	= lopDONE;
    //------------------------------------------------------------------//
}

int  CCdlCOM::send(const void* _buf, int _size) {
    //------------------------------------------------------------------//
    if (prtHdl < 0) {
	lop = lopFAIL;
	return 0;
    } tcflush(prtHdl,TCIFLUSH);
    //------------------------------------------------------------------//
    int readsz = write(prtHdl, _buf, _size);
    if (readsz != _size) {
	lop = lopFAIL;
	return readsz;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;
    return readsz;
    //------------------------------------------------------------------//
}

int  CCdlCOM::recv(void* _buf, int _size) {
    //------------------------------------------------------------------//
    unsigned char*	tmp	= (unsigned char*)_buf;
    register int	readsz 	= 0;
    static double	dt = 0.0f;
	
    //------------------------------------------------------------------//
    if (prtHdl < 0) {
	lop = lopFAIL;
	return 0;
    }
    //------------------------------------------------------------------//
    /*long timer = clock();
      while ((readsz<_size) && (clock() - timer < ccd.rttc)) {
      readsz += read(prtHdl, &tmp[readsz], _size-readsz);
      }*/
    //------------------------------------------------------------------//
    double timer = double(clock());
    while (readsz<_size && dt<ccd.rttc) {
	readsz  = readsz + read(prtHdl, &tmp[readsz], _size-readsz);
	dt      = 1000.0f * (clock()-timer) / double(CLOCKS_PER_SEC);
    }
    //------------------------------------------------------------------//
    if (readsz != _size) {
	lop = lopFAIL;
	return readsz;
    }
    //------------------------------------------------------------------//
    lop = lopDONE;
    tcflush(prtHdl,TCIFLUSH);
    return readsz;
    //------------------------------------------------------------------//
}


//-------------------------------------------------------//
#endif //WIN32 else LINUX
//-------------------------------------------------------//

/****************************************************************************/
