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
#include "KNI/kmlBase.h"
#include "KNI/functions.h"
//#include <memory>
#include <string>
//#include <iostream>

/****************************************************************************/

bool CKatBase::init(
    const TKatGNL _gnl,
    const TKatMOT _mot,
    const TKatSCT _sct,
    const TKatEFF _eff,
    CCplBase* _protocol) {

    //init vars
    gnl = _gnl;
    mot = _mot;
    sct = _sct;
    eff = _eff;

    protocol = _protocol;

    //init motors
    mot.arr = new CMotBase[mot.cnt];
    for (int m=0; m<mot.cnt; m++) {
	if (!mot.arr[m].init(this,mot.desc[m],protocol)) {
	    for (int j=0; j<m; j++) {
		mot.arr[j].free();
	    } return false;
	}
    }

	//reserve memory space to store position of the motors when they are read simultaneously
	mps.pos = new short[mot.cnt];

    //check out whether the motors use new parameters
    if (recvNMP() != RECV_NO_ERR) return false;

    //init sensor contollers
    sct.arr = new CSctBase[sct.cnt];
    for (int s=0; s<sct.cnt; s++) {
	if (!sct.arr[s].init(this,sct.desc[s],protocol)) {
	    for (int j=0; j<s; j++) {
		sct.arr[j].free();
	    } return false;
	}
    }

    return true;
}

void CKatBase::free() {
    delete[] mot.arr;
    delete[] sct.arr;
}

//--------------------------------------------------------------------------//

TRetRecvP CKatBase::recvMFW() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'B';

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    mfw.ver = buf[1];
	    mfw.rev = buf[2];
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CKatBase::recvIDS() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'Y';

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    memcpy(ids.strID,buf+1,sz-1);
	    ids.strID[sz-3] = 0;
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CKatBase::recvCTB() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'X';

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    memcpy(ctb.cmdtbl,buf+1,sz-1);
	    ctb.cmdtbl[sz-1] = 0;
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

//'G'et every 'M'otor's 'S'tatus flag
TRetRecvP CKatBase::recvGMS() {
    int i;
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'N';
    p[1] = 1;
    p[2] = 0;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    for (i=0; i<mot.cnt; i++) {
		mot.arr[i].pvp.msf = (TMotStsFlg)buf[i+1];
	    }; return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CKatBase::recvECH() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'Z';

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    ech.echo =  buf[0];
	    if (ech.echo != 'z') {
		return RECV_READING_ERR;
	    } return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CKatBase::recvCBX() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'U';
    p[1] = 0;
    p[2] = 0;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    cbx.inp[0] = !(buf[2] & 4);
	    cbx.inp[1] = !(buf[2] & 32);
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetSetP CKatBase::sendCBX(const TKatCBX* _cbx) {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'U';
    p[1] = 0;

    p[2] = _cbx->out[0] ? 52 : 36;
    switch (protocol->comm(p,buf,&sz)) {
	case COMM_NO_ERR:	break;
	default:		return SET_WRITTING_ERR;
    }

    p[2] = _cbx->out[1] ? 53 : 37;
    switch (protocol->comm(p,buf,&sz)) {
	case COMM_NO_ERR:	break;
	default:		return SET_WRITTING_ERR;
    }

    cbx.out[0] = _cbx->out[0];
    cbx.out[1] = _cbx->out[1];
    return SET_NO_ERR;
}

TRetSetP CKatBase::sendTPSP() {

    byte	p[32];					//packet
    byte	buf[256];				//readbuf
    byte	sz = 0;					//readbuf size
    byte	wm = 0;					// which motor should be moved
    TMotCmdFlg comm_flag = MCF_ON;	// motor status after moving
    byte	max_accel = 1;			// max_accel

    for (int i=0; i<5; i++) {		// parallel movements only work for 5				
	if (mot.arr[i].freedom)		// motors
	    wm |= 0x01 << i;		// sets the motors that should be moved by
	// setting the proper bits
    };

    p[0] = 'P';						// command
    p[1] = 1;						// always 1
    p[2] = wm;						// which motor should be moved
    p[3] = comm_flag;				// motor status after moving
    p[4] = max_accel;

    for (int i=1; i<=5; i++) {		
	p[4*i+1] = (byte)(mot.arr[i-1].GetTPS()->tarpos >> 8);	
	p[4*i+2] = (byte)(mot.arr[i-1].GetTPS()->tarpos);
	if (mot.arr[i-1].nmp) { // check whether the motor use the new parameters
	    p[4*i+3] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed_nmp >> 8);
	    p[4*i+4] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed_nmp);
	} else {
	    p[4*i+3] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed >> 8);
	    p[4*i+4] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed);
	};
    }

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    //if (!buf[1])	return SET_WRITTING_ERR;

	    return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
    }
}

TRetRecvP CKatBase::recvNMP() {

    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size
    p[0] = 'A';
    p[1] = 0;
    p[2] = 0;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_RTOERR:
	    for (int i=0; i<mot.cnt; i++) {
		mot.arr[i].nmp = false;
	    }
	    //mot.nmp = false;
	    return RECV_NO_ERR;

	case COMM_NO_ERR:
	    for (int i=0; i<mot.cnt; i++) {
		mot.arr[i].nmp = true;
	    }
	    return RECV_NO_ERR;

	default:	return RECV_READING_ERR;
    }
}

TRetSetP CKatBase::enableCrashLimits() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'A';
	p[1] = 1;
	p[2] = 1;

	switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
		//if (!buf[1])	return SET_WRITTING_ERR;

		return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
	}
}

TRetSetP CKatBase::disableCrashLimits() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'A';
	p[1] = 0;
	p[2] = 0;

	switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
		//if (!buf[1])	return SET_WRITTING_ERR;

		return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
	}
}

TRetSetP CKatBase::setCrashLimit(long idx, int limit) {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'S';
	p[1] = 5;			// subcommand 5 "Set Crashlimit"
	p[2] = (char)(limit >> 8);
	p[3] = (char)(limit);
	p[4] = 0;

	switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
		//if (!buf[1])	return SET_WRITTING_ERR;

		return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
	}
}

TRetSetP CKatBase::unBlock() {

	bool unblock_OK = true;

	for (int i=0; i<mot.cnt; i++) {
		unblock_OK &= (mot.arr[i].resetBlocked() == SET_NO_ERR);
	}

	if (unblock_OK) return SET_NO_ERR;

	return SET_WRITTING_ERR;
}

TRetSetP CKatBase::sendSLMP(byte* p) {
    // Set Linear movements parameters

    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    //if (!buf[1])	return SET_WRITTING_ERR;

	    return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
    }
}

TRetSetP CKatBase::sendSLM(bool exactflag) {
    // Start Linear movement

    byte	p[3];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 204; // 'L'+128
    p[1] = 1;
    p[2] = exactflag ? 1 : 0;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    //if (!buf[1])	return SET_WRITTING_ERR;

	    return SET_NO_ERR;

	case COMM_RTOERR:
	    printf("\nCOMM_RTOERR");
	    return SET_WRITTING_ERR;		

	case COMM_CRCERR:
	    printf("\nCOMM_CRCERR");
	    return SET_WRITTING_ERR;	

	default:
	    return SET_WRITTING_ERR;
    }
}

TRetRecvP CKatBase::recvMPS() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'N';
	p[1] = 3;
	p[2] = 0;

	if (protocol->comm(p,buf,&sz)==COMM_NO_ERR) {

			for (int i=0; i<mot.cnt; i++) {
				mps.pos[i] = (((short)buf[2*i+1]) <<8) | buf[2*i+2];
			};

			return RECV_NO_ERR;
	}

	return RECV_READING_ERR;
	
}



/****************************************************************************/
/****************************************************************************/

bool CMotBase::init(CKatBase* _own, const TMotDesc _motDesc, CCplBase* _protocol) {
    gnl.own = _own;
    gnl.SID = _motDesc.slvID;
    protocol =  _protocol;
    return true;
}

void CMotBase::free() {
}

//--------------------------------------------------------------------------//

TRetSetP CMotBase::resetBlocked() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	if (recvPVP() != RECV_NO_ERR) 
		return SET_WRITTING_ERR;

	p[0] = 'C';
	p[1] = gnl.SID;
	p[2] = MCF_FREEZE;			// Flag to freeze
	p[3] = (byte)(GetPVP()->pos >> 8);
	p[4] = (byte)(GetPVP()->pos);

	switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
		aps.mcfAPS = MCF_FREEZE;	return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
	}
}

TRetSetP CMotBase::sendAPS(const TMotAPS* _aps) {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'C';
    p[1] = gnl.SID + 128;
    p[2] = _aps->mcfAPS;
    p[3] = (byte)(_aps->actpos >> 8);
    p[4] = (byte)(_aps->actpos);

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1])	return SET_WRITTING_ERR;
	    aps = *_aps;	return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
    }
}

TRetSetP CMotBase::sendTPS(const TMotTPS* _tps) {

    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'C';
    p[1] = gnl.SID;
    p[2] = _tps->mcfTPS;
    p[3] = (byte)(_tps->tarpos >> 8);
    p[4] = (byte)(_tps->tarpos);

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1])	return SET_WRITTING_ERR;
	    tps = *_tps;	return SET_NO_ERR;

	default:		return SET_WRITTING_ERR;
    }
}

TRetSetP CMotBase::sendSCP(const TMotSCP* _scp) {

    // If the parameters are set wrong, this method retries "number_of_retries" 
    // times to set them

    const TMotSCP* P_SCP;
    short number_of_retries = 3;

    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    if (nmp) {

	// ----------------- New motor parameters ------------------ //
	while (number_of_retries-- > 0) {

	    p[0] = 'K';
	    p[1] = gnl.SID;
	    p[2] = _scp->maxppwm_nmp;
	    p[3] = _scp->maxnpwm_nmp;
	    p[4] = _scp->kspeed_nmp;
	    p[5] = _scp->kpos_nmp;
	    p[6] = _scp->kI_nmp;
	    p[7] = _scp->crash_limit_nmp >> 8;
	    p[8] = _scp->crash_limit_nmp;
	    p[9] = _scp->crash_limit_lin_nmp >> 8;
	    p[10]= _scp->crash_limit_lin_nmp;	

	    if (protocol->comm(p,buf,&sz) == COMM_NO_ERR) {

		if (this->recvSCP() == RECV_READING_ERR) {
		    return SET_READING_ERR;
		};

		P_SCP = this->GetSCP();

		if (P_SCP->maxppwm_nmp ==_scp->maxppwm_nmp && P_SCP->maxnpwm_nmp ==_scp->maxnpwm_nmp && 
		    P_SCP->kspeed_nmp ==_scp->kspeed_nmp && P_SCP->kpos_nmp ==_scp->kpos_nmp && P_SCP->kI_nmp ==_scp->kI_nmp) {
		    scp = *_scp;
		    return SET_NO_ERR;

		};

	    } else {
		return SET_WRITTING_ERR;
	    }; 
	} // end while

    } else {

	// ----------------- Old motor parameters ------------------ //
	while (number_of_retries-- > 0) {

	    p[0] = 'K';
	    p[1] = gnl.SID;
	    p[2] = _scp->maxppwm;
	    p[3] = _scp->maxnpwm;
	    p[4] = _scp->kP;
	    p[5] = _scp->kI;
	    p[6] = _scp->kD;
	    p[7] = _scp->kARW;
	    p[8] = _scp->kP_speed;
	    p[9] = _scp->kI_speed;
	    p[10]= _scp->kD_speed;	

	    if (protocol->comm(p,buf,&sz) == COMM_NO_ERR) {

		if (this->recvSCP() == RECV_READING_ERR) {
		    return SET_READING_ERR;
		};

		P_SCP = this->GetSCP();

		// old firmwares loaded on PIC16 set by default maxppwm and maxnpwm to 70 regardless of the value
		// sent with the command 'k' (actually is set to 70 when this value is higher than 70).

		if ((P_SCP->maxppwm ==_scp->maxppwm || P_SCP->maxppwm == 70) && (P_SCP->maxnpwm ==_scp->maxnpwm || P_SCP->maxnpwm == 70) && 
		    P_SCP->kP ==_scp->kP && P_SCP->kI ==_scp->kI && P_SCP->kD ==_scp->kD &&
		    P_SCP->kP_speed ==_scp->kP_speed) {
		    scp = *_scp;
		    return SET_NO_ERR;
		};

	    } else {
		return SET_WRITTING_ERR;
	    }; 

	}; // end while

    }; // end if

    return SET_WRONG_PARAMETERS;
}

TRetRecvP CMotBase::recvSCP() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'V';
    p[1] = gnl.SID;
    p[2] = 1;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1])	return RECV_READING_ERR;

	    if (nmp) {
		scp.maxppwm_nmp			= buf[3];	
		scp.maxnpwm_nmp			= buf[4];
		scp.kspeed_nmp			= buf[5];
		scp.kpos_nmp			= buf[6];
		scp.kI_nmp				= buf[7];
		scp.crash_limit_nmp		= buf[8];
		scp.crash_limit_nmp		<<= 8;
		scp.crash_limit_nmp		+= buf[9];
		scp.crash_limit_lin_nmp	= buf[10];	
		scp.crash_limit_lin_nmp	<<= 8;
		scp.crash_limit_lin_nmp	+= buf[11];

	    } else {
		scp.maxppwm	= buf[3];
		scp.maxnpwm	= buf[4];
		scp.kP		= buf[5];
		scp.kI		= buf[6];
		scp.kD		= buf[7];
		scp.kARW	= buf[8];
		scp.kP_speed= buf[9];
		scp.kI_speed= buf[10];
		scp.kD_speed= buf[11];
	    };

	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetSetP CMotBase::sendDYL(const TMotDYL* _dyl) {

    // If the parameters are set wrong, this method retries "number_of_retries" 
    // times to set them

    const TMotDYL* P_DYL;
    short number_of_retries = 3;

    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size


    if (nmp) {

	while (number_of_retries-- > 0) {

	    p[0] = 'O';
	    p[1] = gnl.SID;
	    p[2] = _dyl->maxaccel_nmp;
	    p[3] = 0;
	    p[6] = (byte)(_dyl->maxpspeed_nmp >> 8);
	    p[7] = (byte)(_dyl->maxpspeed_nmp);
	    p[8] = (byte)(_dyl->maxnspeed_nmp >> 8);
	    p[9] = (byte)(_dyl->maxnspeed_nmp);
	    p[10] = (byte) 0;				
	    p[11] = (byte) _dyl->maxcurr_nmp;	

	    if (protocol->comm(p,buf,&sz) == COMM_NO_ERR) {

		if (this->recvDYL() == RECV_READING_ERR) {
		    return SET_READING_ERR;
		};

		P_DYL = this->GetDYL();

		if (P_DYL->maxaccel_nmp ==_dyl->maxaccel_nmp && P_DYL->maxpspeed_nmp ==_dyl->maxpspeed_nmp && 
		    P_DYL->maxnspeed_nmp ==_dyl->maxnspeed_nmp) {
		    dyl = *_dyl;
		    return SET_NO_ERR;
		};

	    } else {
		return SET_WRITTING_ERR;
	    };
	}; // end while

    } else {

	while (number_of_retries-- > 0) {
	
	    // Package format compatible since firmware version 40.2 for masterboard's PIC16 
	    // and version 13.5 for PIC16 microcontrolers within the slave boards
	    // Compatible with all the firmwares for PIC18 with PID parameters

	    p[0] = 'O';
	    p[1] = gnl.SID;
	    p[2] = _dyl->maxaccel;
	    p[3] = _dyl->maxdecel;
	    p[6] = (byte)(_dyl->maxpspeed >> 8);
	    p[7] = (byte)(_dyl->maxpspeed);
	    p[8] = (byte)(_dyl->maxnspeed >> 8);
	    p[9] = (byte)(_dyl->maxnspeed);
	    p[10] = (byte) 0;				
	    p[11] = (byte) _dyl->maxcurr;	

	    if (protocol->comm(p,buf,&sz) == COMM_NO_ERR) {

		if (this->recvDYL() == RECV_READING_ERR) {
		    return SET_READING_ERR;
		};

		P_DYL = this->GetDYL();

		if (P_DYL->maxcurr ==_dyl->maxcurr && P_DYL->maxnspeed ==_dyl->maxnspeed && 
		    P_DYL->maxpspeed ==_dyl->maxpspeed) {
		    dyl = *_dyl;
		    return SET_NO_ERR;
		};

	    } else {
		return SET_WRITTING_ERR;
	    };
	}; // end while

    }; // end if


    return SET_WRONG_PARAMETERS;
}

TRetRecvP CMotBase::recvDYL() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'V';
    p[1] = gnl.SID;
    p[2] = 2;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:

	    if (!buf[1])	return RECV_READING_ERR;

	    if (nmp) {

		dyl.maxaccel_nmp	= buf[3];
		dyl.maxpspeed_nmp	= (((short)buf[5]) <<8) | buf[6];
		dyl.maxnspeed_nmp	= (((short)buf[7]) <<8) | buf[8];
		dyl.maxcurr_nmp		= buf[9];

	    } else {

		// Package format compatible since firmware version 40.2 for masterboard's PIC16 
		// and version 13.5 for PIC16 microcontrolers within the slave boards
		// Compatible with all the firmwares for PIC18 with PID parameters

		dyl.maxaccel	= buf[3];
		dyl.maxdecel	= 0;
		dyl.minpos		= 0;
		dyl.maxpspeed	= (((short)buf[7]) <<8) | buf[8];
		dyl.maxnspeed	= (((short)buf[9]) <<8) | buf[10];
		dyl.maxcurr		= buf[11];
		dyl.actcurr		= 0;

	    };

	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CMotBase::recvPVP() {

    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'D';
    p[1] = gnl.SID;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1])	return RECV_READING_ERR;
	    pvp.msf		= (TMotStsFlg)buf[2];
	    pvp.pos		= (((short)buf[3])<<8) | buf[4];
	    pvp.vel		= (((short)buf[5])<<8) | buf[6];
	    pvp.pwm		= buf[7];
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

TRetRecvP CMotBase::recvSFW() {
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    p[0] = 'V';
    p[1] = gnl.SID;
    p[2] = 32;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1])	return RECV_READING_ERR;
	    sfw.version	= buf[3];
	    sfw.subversion	= buf[4];
	    sfw.revision	= buf[5];
	    sfw.type	= buf[6];
	    sfw.subtype	= buf[7];
	    return RECV_NO_ERR;

	default:		return RECV_READING_ERR;
    }
}

void CMotBase::setTPSP(int _tar) {
    tps.tarpos = _tar;
    freedom = true;
}

void CMotBase::resetTPSP() {
    freedom = false;
}

bool CMotBase::setENL(TEncUnit _enc_range, 
		      TEncUnit _enc_minpos, 
		      TEncUnit _enc_maxpos, 
		      TEncUnit _enc_per_cicle,
		      TEncUnit _enc_tolerance) {

    if (_enc_range < 0) return false;

    enl.enc_range		= _enc_range;
    enl.enc_minpos		= _enc_minpos;
    enl.enc_maxpos		= _enc_maxpos;
    enl.enc_per_cicle	= _enc_per_cicle;
    enl.enc_tolerance	= _enc_tolerance;

    return true;
}

bool CMotBase::checkENL(TEncUnit _enc_value) {

    return (_enc_value > enl.enc_minpos && _enc_value < enl.enc_maxpos);

}


TRetMOV CMotBase::inc(TEncUnit dif, bool wait, long timeout) {
    if (recvPVP() != RECV_NO_ERR) {
	return MOV_RECVRMP_FAILED;
    };
    return mov((TEncUnit)(GetPVP()->pos + dif),
	       wait, timeout);
}

TRetMOV CMotBase::dec(TEncUnit dif, bool wait, long timeout) {
    if (recvPVP() != RECV_NO_ERR) {
	return MOV_RECVRMP_FAILED;
    };
    return mov((TEncUnit)(GetPVP()->pos - dif),
	       wait, timeout);
}

TRetMOV CMotBase::mov(TEncUnit tar, bool wait, long timeout) {

    if (!checkENL(tar))		return MOV_RANGE_ERROR;

    tps.mcfTPS = MCF_ON;
    tps.tarpos = tar;
    if (sendTPS(&tps) != SET_NO_ERR) {
	return MOV_SENDTPS_FAILED;
    }

    if (!wait) return MOV_TARGET_SET;

    clock_t t = gettics();	
    while (true) {
	if ((1000*((gettics()-t)/CLOCKS_PER_SEC) > timeout) && (timeout > 0)){
	    return MOV_TIMEOUT;
	}
	//-------------------------------------------------------------//
	//SLEEP(2);	//give enough time for the katana to respond
	//-------------------------------------------------------------//
	if (recvPVP() != RECV_NO_ERR) {
	    return MOV_RECVRMP_FAILED;
	}
	//-------------------------------------------------------------//
	if (ABSOLUTE(tar - GetPVP()->pos) <
	    enl.enc_tolerance) {
	    return MOV_POSITION_REACHED;
	}
	//-------------------------------------------------------------//
	if (GetPVP()->msf == 40)
		return MOV_CRASHED;
	//-------------------------------------------------------------//

    }

    return MOV_TARGET_SET;
}


/****************************************************************************/
/****************************************************************************/

bool CSctBase::init(CKatBase* _own, const TSctDesc _sctDesc, CCplBase* _protocol) {
    //printf("CSctBase::init()\n");
    gnl.own = _own;
    gnl.SID = _sctDesc.ctrlID;
    gnl.res = _sctDesc.sens_res;
    dat.cnt = _sctDesc.sens_count;
    dat.arr = new short[dat.cnt];
    protocol =  _protocol;
    return true;
}

void CSctBase::free() {
    //delete[] dat.arr;
};

TRetCMD CSctBase::recvDAT() {
    int i;			//iterator
    byte	p[32];		//packet
    byte	buf[256];	//readbuf
    byte	sz = 0;		//readbuf size

    //switch between 8/12 bit resolution
    p[0] = (gnl.res != 12) ? 'E' : 'E' + 128;
    p[1] = gnl.SID;

    switch (protocol->comm(p,buf,&sz)) {

	case COMM_NO_ERR:
	    if (!buf[1]) return SLVERR;
	    for (i=0; i<dat.cnt; i++) {
		dat.arr[i] = (short)buf[i+2];
		//printf("%d ",dat.arr[i]);
	    } return NO_ERR;

	case COMM_RTOERR:	return RTOERR;
	case COMM_CRCERR:	return CRCERR;
	default:		return NO_ERR;
    }
}

/****************************************************************************/
