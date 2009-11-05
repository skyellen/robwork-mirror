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
#include "KNI/kmlExt.h"
#include "KNI/functions.h"
/****************************************************************************/
#include <time.h>
//#include <memory>
#include <string>
#include <fstream>
//#include <iostream>
/****************************************************************************/
using namespace std;
/*--------------------------------------------------------------------------*/

//TODO change to c
bool readEntry(ifstream& file, char* dest, int destsz,
	       const char* section,
	       const char* subsection,
	       const char* entry) {

    char line[256];
    short pos = 0;
    short idx = 0;

    file.seekg(0); //goto the begin

    do {	//search section
	if (!file.good()) goto failed;
	memset(line,0,sizeof(line));
	file.getline(line, sizeof(line));
	strtok(line,"\r"); // strip off the CR
    } while (strcmp(line,section));

    do {	//search subsection
	if (!file.good()) goto failed;
	memset(line,0,sizeof(line));
	file.getline(line, sizeof(line));
	strtok(line,"\r"); // strip off the CR
    } while (strcmp(line,subsection));

    do {	//search entry
	if (!file.good()) goto failed;
	memset(line,0,sizeof(line));
	file.getline(line, sizeof(line));
	strtok(line,"\r"); // strip off the CR
    } while (strncmp(line,entry,strlen(entry)));

    //parse input line the detect entry value

    while (line[pos++] != '=') {
	if (pos == 256)	goto failed;
    }
    while (line[pos++] != '"') {
	if (pos == 256)	goto failed;
    }

    memset(dest,0,destsz);
    while (line[pos] != '"') {
	dest[idx++] = line[pos++];
	if (pos == 256)	goto failed;
    }
    while (line[pos++] != ';') {
	if (pos == 256)	goto failed;
    } return true;

 failed:
    return false;
}


/*--------------------------------------------------------------------------*/

void CKatana::default_map(TMotMAP* map) {
    //--------------------------------------------------------------//
    map->enc_range		= 32768;
    map->enc_maxpos		=  map->enc_range / 2;
    map->enc_minpos		= -map->enc_range / 2;;

    map->map_maxpos		= (double)map->enc_maxpos;
    map->map_minpos		= (double)map->enc_minpos;
    map->map_range		= map->map_maxpos - map->map_minpos;

    map->enc_to_map	= (double)(map->map_range / map->enc_range);
    map->map_to_enc	= (double)(map->enc_range / map->map_range);
    //--------------------------------------------------------------//
    //map->enc_tolerance	= 50;	//<<-- IMPORTANT!! (precision)
    map->calib			= false;//will be set true after calibration
    //--------------------------------------------------------------//
}

/*--------------------------------------------------------------------------*/

TRetMOV CKatana::inc(long idx, TMapUnit dif, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].inc((TEncUnit)(dif*map[idx].map_to_enc),wait,timeout);
}

TRetMOV CKatana::dec(long idx, TMapUnit dif, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].dec((TEncUnit)(dif*map[idx].map_to_enc),wait,timeout);
}

TRetMOV CKatana::mov(long idx, TMapUnit tar, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].mov((TEncUnit)
					((tar-map[idx].map_minpos) * map[idx].map_to_enc 
					 + map[idx].enc_minpos),
					wait,timeout);
}

/*--------------------------------------------------------------------------*/

TRetMOV CKatana::incDegrees(long idx, double dif, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].inc(degreetoencoder(idx, dif),wait,timeout);
}

TRetMOV CKatana::decDegrees(long idx, double dif, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].dec(degreetoencoder(idx, dif),wait,timeout);
}

TRetMOV CKatana::movDegrees(long idx, double tar, bool wait, long timeout) {
    return base->GetMOT()->arr[idx].mov(degreetoencoder(idx, tar),
					wait,timeout);
}

/*--------------------------------------------------------------------------*/

void CKatana::setTPSP(long idx, TMapUnit _tar) {

    base->GetMOT()->arr[idx].setTPSP(maptoencoder(idx, _tar)); 

}

void CKatana::resetTPSP() {

    for (int i=0; i < base->GetMOT()->cnt ; i++) 
	base->GetMOT()->arr[i].resetTPSP();
    
}

TRetMOV CKatana::sendTPSP(bool wait, long timeout) {

    bool pos_reached;

    if (base->sendTPSP() == SET_NO_ERR) {

	if (!wait) return MOV_TARGET_SET;

	clock_t t = gettics();
	while (true) {

	    if ((1000*((gettics()-t)/CLOCKS_PER_SEC) > timeout ) && (timeout > 0)){
		return MOV_TIMEOUT;
	    }
	    //-------------------------------------------------------------//
	    SLEEP(2);	//give enough time for the katana to respond
			//-------------------------------------------------------------//

	    pos_reached = true;
	    for (int idx=0; idx<5; idx++) {

		if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) {
		    return MOV_RECVRMP_FAILED;
		}

		pos_reached &= !base->GetMOT()->arr[idx].GetFreedom() || 
		    ABSOLUTE(base->GetMOT()->arr[idx].GetTPS()->tarpos - base->GetMOT()->arr[idx].GetPVP()->pos)
		    < base->GetMOT()->arr[idx].GetENL()->enc_tolerance;
	    }

	    if (pos_reached) {

		// clear the freedom flag of each motor
		for (int idx=0; idx<5; idx++) {					
		    base->GetMOT()->arr[idx].resetTPSP();
		};
		return MOV_POSITION_REACHED;
	    };

	    //-------------------------------------------------------------//
	}		
			
    }

    return MOV_SENDTPS_FAILED;

}

/*--------------------------------------------------------------------------*/

void CKatana::setTPSPDegrees(long idx, double _tar) {

    base->GetMOT()->arr[idx].setTPSP(degreetoencoder(idx, _tar)); 

}

/*--------------------------------------------------------------------------*/

bool CKatana::create(const char* cfgFile, CCplBase* protocol) {
    //----------------------------------------------------------------//
    ifstream file(cfgFile);         //TODO change to c
    if (file.fail()) return false;  //TODO change to c
    //----------------------------------------------------------------//
    char input[256];
    //----------------------------------------------------------------//

    //fill TKatGNL structure ------------------------------------------------------
    if (!readEntry(file,input,sizeof(input),"[KATANA]","[GENERAL]","addr")) {
	return false;
    } TKatGNL general = { atoi(input) };

    //fill TKatMOT structure ------------------------------------------------------
    if (!readEntry(file,input,sizeof(input),"[KATANA]","[GENERAL]","motcnt")) {
	return false;
    } TKatMOT katmot = { atoi(input), 0L, 0L };

    TMotDesc* motdesc = new TMotDesc[katmot.cnt];
    for (int i=0; i<katmot.cnt; i++) {
	char section[256];
	memset(section,0,sizeof(section));
	sprintf(section,"[MOT[%d]]",i);
	if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","slvID")) {
	    return false;
	} motdesc[i].slvID = atoi(input);
    } katmot.desc = motdesc;

    //fill TKatSCT structure ------------------------------------------------------
    if (!readEntry(file,input,sizeof(input),"[KATANA]","[GENERAL]","sctcnt")) {
	return false;
    } TKatSCT katsct = { atoi(input), 0L, 0L };

    TSctDesc* sctdesc = new TSctDesc[katsct.cnt];
    for (int j=0; j<katsct.cnt; j++) {
	char section[256];
	memset(section,0,sizeof(section));
	sprintf(section,"[SCT[%d]]",j);
	if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","ctrlID")) {
	    return false;
	} sctdesc[j].ctrlID = atoi(input);
	if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","sens_res")) {
	    return false;
	} sctdesc[j].sens_res = atoi(input);
	if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","sens_count")) {
	    return false;
	} sctdesc[j].sens_count = atoi(input);
    } katsct.desc = sctdesc;

    //fill TKatEFF structure ------------------------------------------------------
    TKatEFF eff;
    char section[256];
    memset(section,0,sizeof(section));
    sprintf(section,"[ENDEFFECTOR]");
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","segment1")) {
	return false;
    } eff.arr_segment[0] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","segment2")) {
	return false;
    } eff.arr_segment[1] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","segment3")) {
	return false;
    } eff.arr_segment[2] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","segment4")) {
	return false;
    } eff.arr_segment[3] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta1")) {
	return false;
    } eff.arr_delta[0] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta2")) {
	return false;
    } eff.arr_delta[1] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta3")) {
	return false;
    } eff.arr_delta[2] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta4")) {
	return false;
    } eff.arr_delta[3] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta5")) {
	return false;
    } eff.arr_delta[4] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","delta6")) {
	return false;
    } eff.arr_delta[5] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","x_position")) {
	return false;
    } eff.arr_position[0] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","y_position")) {
	return false;
    } eff.arr_position[1] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","z_position")) {
	return false;
    } eff.arr_position[2] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","alfa")) {
	return false;
    } eff.arr_orientation[0] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","beta")) {
	return false;
    } eff.arr_orientation[1] =  atof(input) ;
    if (!readEntry(file,input,sizeof(input),section,"[GENERAL]","gamma")) {
	return false;
    } eff.arr_orientation[2] =  atof(input) ;

	//close file
	//TODO ..
	
    //create the katana robot class ----------------------------------------------
    return create(general,katmot,katsct,eff,protocol);
}

bool CKatana::create(TKatGNL& gnl, TKatMOT& mot, TKatSCT& sct, TKatEFF& eff,
		     CCplBase* protocol) {
    //----------------------------------------------------------------//
    map = new TMotMAP[mot.cnt];
	
    //----------------------------------------------------------------//
    return base->init(gnl,mot,sct,eff,protocol);
    //----------------------------------------------------------------//
}

/*--------------------------------------------------------------------------*/

bool CKatana::calibrate(const char*	cfgFile) {
    //----------------------------------------------------------------//
    ifstream file(cfgFile);
    if (file.fail()) return false;
    //----------------------------------------------------------------//
    char input[256];
    //----------------------------------------------------------------//
    //set motors ON before calibrating
    //----------------------------------------------------------------//
    TMotAPS aps; 
    for (int i=0; i<base->GetMOT()->cnt; i++) { 
	aps.actpos = 0;		
	aps.mcfAPS = MCF_ON;	
	if (base->GetMOT()->arr[i].sendAPS(&aps) != SET_NO_ERR) {
	    return false;
	};
    };
    //----------------------------------------------------------------//
    TMotCLB* clbarr = new TMotCLB[base->GetMOT()->cnt];
    for (int i=0; i<base->GetMOT()->cnt; i++) {
	char section[256];
	memset(section,0,sizeof(section));
	sprintf(section,"[MOT[%d]]",i);

	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","enable")) {
	    return false;
	} clbarr[i].enable = strcmp("TRUE",input) ? false : true;
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","dir")) {
	    return false;
	} clbarr[i].dir = strcmp("DIR_POSITIVE",input) ? DIR_NEGATIVE : DIR_POSITIVE;
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","diff")) {
	    return false;
	} clbarr[i].diff = atoi(input);
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","mcf")) {
	    return false;
	}
	if (!strcmp("MCF_OFF",   input)) clbarr[i].mcf = MCF_OFF;
	if (!strcmp("MCF_ON",    input)) clbarr[i].mcf = MCF_ON;
	if (!strcmp("MCF_FREEZE",input)) clbarr[i].mcf = MCF_FREEZE;
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","enc_range")) {
	    return false;
	} clbarr[i].enc_range  = atoi(input);
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","timeout")) {
	    return false;
	} clbarr[i].timeout = strcmp("TM_ENDLESS", input) ? atoi(input) : TM_ENDLESS;
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","map_minpos")) {
	    return false;
	} clbarr[i].map_minpos = atof(input);

	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","map_maxpos")) {
	    return false;
	} clbarr[i].map_maxpos = atof(input);
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","enc_per_circle")) {
	    return false;
	} clbarr[i].enc_per_circle = atol(input);
	//----------------------------------------------------------------------------//
	if (!readEntry(file,input,sizeof(input),section,"[CALIBRATION]","enc_tolerance")) {
	    return false;
	} clbarr[i].enc_tolerance  = atoi(input);
	//----------------------------------------------------------------------------//
    }

    //----------------------------------------------------------------//
    TMotSCP* scparr = new TMotSCP[base->GetMOT()->cnt];
    for (int j=0; j<base->GetMOT()->cnt; j++) {
	char section[256];
	memset(section,0,sizeof(section));
	sprintf(section,"[MOT[%d]]",j);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","maxppwm")) {
	    return false;
	} scparr[j].maxppwm = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","maxnpwm")) {
	    return false;
	} scparr[j].maxnpwm = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kP")) {
	    return false;
	} scparr[j].kP = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kI")) {
	    return false;
	} scparr[j].kI = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kD")) {
	    return false;
	} scparr[j].kD = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kARW")) {
	    return false;
	} scparr[j].kARW = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kP_speed")) {
	    return false;
	} scparr[j].kP_speed = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kI_speed")) {
	    return false;
	} scparr[j].kI_speed = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kD_speed")) {
	    return false;
	} scparr[j].kD_speed = atoi(input);


	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","maxppwm_nmp")) {
	    return false;
	} scparr[j].maxppwm_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","maxnpwm_nmp")) {
	    return false;
	} scparr[j].maxnpwm_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kspeed_nmp")) {
	    return false;
	} scparr[j].kspeed_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kpos_nmp")) {
	    return false;
	} scparr[j].kpos_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","kI_nmp")) {
	    return false;
	} scparr[j].kI_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","crash_limit_nmp")) {
	    return false;
	} scparr[j].crash_limit_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[STATIC]","crash_limit_lin_nmp")) {
	    return false;
	} scparr[j].crash_limit_lin_nmp = atoi(input);

    }

    //----------------------------------------------------------------//
    TMotDYL* dylarr = new TMotDYL[base->GetMOT()->cnt];
    for (int k=0; k<base->GetMOT()->cnt; k++) {
	char section[256];
	memset(section,0,sizeof(section));
	sprintf(section,"[MOT[%d]]",k);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxaccel")) {
	    return false;
	} dylarr[k].maxaccel = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxdecel")) {
	    return false;
	} dylarr[k].maxdecel = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","minpos")) {
	    return false;
	} dylarr[k].minpos = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxpspeed")) {
	    return false;
	} dylarr[k].maxpspeed = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxnspeed")) {
	    return false;
	} dylarr[k].maxnspeed = atoi(input);

	/*if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxpcurr")) {
	  return false;
	  } dylarr[k].maxpcurr = atoi(input);

	  if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxncurr")) {
	  return false;
	  } dylarr[k].maxncurr = atoi(input);*/

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxcurr")) {
	    return false;
	} dylarr[k].maxcurr = atoi(input);

	dylarr[k].actcurr = 0;


	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxaccel_nmp")) {
	    return false;
	} dylarr[k].maxaccel_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxpspeed_nmp")) {
	    return false;
	} dylarr[k].maxpspeed_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxnspeed_nmp")) {
	    return false;
	} dylarr[k].maxnspeed_nmp = atoi(input);

	if (!readEntry(file,input,sizeof(input),section,"[DYNAMIC]","maxcurr_nmp")) {
	    return false;
	} dylarr[k].maxcurr_nmp = atoi(input);
    }

    //----------------------------------------------------------------//
    TKatCLB katclb = { base->GetMOT()->cnt, clbarr, scparr, dylarr };
    return calibrate(&katclb);
}

/*--------------------------------------------------------------------------*/

bool CKatana::searchMechStop(long idx, TSearchDir dir, TEncUnit dif,
			     long timeout, TMotSCP _scp, TMotDYL _dyl ) {
    /*------------------------------------------------------------------*/

    // Set parameters for searching the mechanical stoppers
    /*------------------------------------------------------------------*/
    if (base->GetMOT()->arr[idx].sendSCP(&_scp) != SET_NO_ERR) {
	return false;
    };

    if (base->GetMOT()->arr[idx].sendDYL(&_dyl) != SET_NO_ERR) {
	return false;
    };

    /*------------------------------------------------------------------*/

    TMotAPS aps;
    switch (dir) {
	case DIR_POSITIVE:
	    aps.actpos = -31000;		// Set the actual position equal to the 
	    aps.mcfAPS = MCF_FREEZE;	// extreme opposite to direction I will move
	    if (base->GetMOT()->arr[idx].sendAPS(&aps) != SET_NO_ERR) {
		return false;
	    }; 
	    break;
	case DIR_NEGATIVE:
	    aps.actpos = 31000;			// Set the actual position equal to the 	
	    aps.mcfAPS = MCF_FREEZE;	// extreme opposite to direction I will move
	    if (base->GetMOT()->arr[idx].sendAPS(&aps) != SET_NO_ERR) {
		return false;
	    };
	    break;
    };

    TMotTPS tps;
    switch (dir) {
	case DIR_POSITIVE:	
	    tps.tarpos = 32000;		
	    tps.mcfTPS = MCF_ON;
	    if (base->GetMOT()->arr[idx].sendTPS(&tps) != SET_NO_ERR) { // Set the target position equal to 
		return false;						// the extreme I am moving towards.
	    } break;
	case DIR_NEGATIVE:
	    tps.tarpos = -32000;		
	    tps.mcfTPS = MCF_ON;
	    if (base->GetMOT()->arr[idx].sendTPS(&tps) != SET_NO_ERR) {// Set the target position equal to
		return false;						// the extreme I am moving towards.
	    } break;
    };

    double average_vel = 50;	// mobile average

    /*------------------------------------------------------------------*/
    //clock_t t = clock();
    /*------------------------------------------------------------------*/

    while (true) {

	/*--------------------------------------------------------------*/
	//if ((1000*(clock()-t) > timeout * CLOCKS_PER_SEC) && (timeout > 0)){
	//	return false;
	//}

	/*--------------------------------------------------------------*/
	// Take 10 samples of the motor velocity and calculate its average.

	const short COUNT = 5;						
	TMotPVP sum; memset(&sum,0,sizeof(sum));

	for (int i=0; i<COUNT; i++) {
	    SLEEP(50);					//millisec
	    if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) {
		return false;
	    };
	    sum.vel += base->GetMOT()->arr[idx].GetPVP()->vel;
	};

	/*--------------------------------------------------------------*/
	double vel = (double)((double)sum.vel / (double)COUNT);
	/*--------------------------------------------------------------*/

	average_vel = (average_vel + ABSOLUTE(vel)) / 2; // Calculate mobile average
	if (average_vel < 1) {							 // for motor velocity

	    // To avoid a compensation on the motor the actual position is set to 0 
	    // Otherwise, as it didn't reach the target position it could attempt to go 
	    // on moving

	    aps.actpos = 0;		 
	    aps.mcfAPS = MCF_FREEZE;	
	    if (base->GetMOT()->arr[idx].sendAPS(&aps) != SET_NO_ERR) 
		return false;

	    return true;	// If it is lower than 1, mech.stop reached
	}

	/*--------------------------------------------------------------*/
    } return false;
    /*----------------------------------------------------------------------*/
}

/*--------------------------------------------------------------------------*/

bool CKatana::calibrate(TKatCLB* clb) {
    bool ret = true;
    for (int i=0; i<clb->cnt; i++) {
	ret &= (calibrate(i,clb->clb[i],clb->scp[i],clb->dyl[i]) == CLB_NO_ERR || !clb->clb[i].enable);
    } return ret;
}

TRetCLB CKatana::calibrate(long idx, TMotCLB clb, TMotSCP scp, TMotDYL dyl) {
    /*-----------------------------------------------------------*/
    //default_map(&map[idx]);

    map[idx].calib			= false;
    map[idx].enc_range		= clb.enc_range;

    map[idx].map_range		= clb.map_maxpos - clb.map_minpos;
    map[idx].map_minpos		= clb.map_minpos;
    map[idx].map_maxpos		= clb.map_maxpos;

    map[idx].enc_to_map		= (double)(map[idx].map_range / map[idx].enc_range);	//calc mapping quot
    map[idx].map_to_enc		= (double)(map[idx].enc_range / map[idx].map_range);	//calc mapping quot
    map[idx].deg_to_enc		= ABSOLUTE((double)((double)clb.enc_per_circle / 360.0));			//calc mapping quot
    map[idx].enc_to_deg		= ABSOLUTE((double)(360.0 / (double)clb.enc_per_circle));			//calc mapping quot

    if (clb.dir == DIR_POSITIVE) {
	map[idx].enc_minpos		= (TEncUnit)(base->GetEFF()->arr_delta[idx] * map[idx].deg_to_enc) - clb.enc_range;
	map[idx].enc_maxpos		= (TEncUnit)(base->GetEFF()->arr_delta[idx] * map[idx].deg_to_enc);
    } else {
	map[idx].enc_minpos		= (TEncUnit)(base->GetEFF()->arr_delta[idx] * map[idx].deg_to_enc);
	map[idx].enc_maxpos		= (TEncUnit)(base->GetEFF()->arr_delta[idx] * map[idx].deg_to_enc) + clb.enc_range ;
    };

    /*-----------------------------------------------------------*/
    base->GetMOT()->arr[idx].setENL(map[idx].enc_range, map[idx].enc_minpos, map[idx].enc_maxpos, clb.enc_per_circle, clb.enc_tolerance);

    /*-----------------------------------------------------------*/
    if (!clb.enable) return CLB_DISABLED;

    /*-----------------------------------------------------------*/
    if (!searchMechStop(idx,clb.dir,clb.diff,clb.timeout,scp,dyl)) {
	return CLB_SEARCHMECHSTOP_ERR;
    } SLEEP(25);

    /*-----------------------------------------------------------*/
    // This patch is necessary because the range of motor 3 is bigger than
    // 65536 and there is and overflow, so the range should be limited.
    // Due to the way how IK is calculated, it is better to set the position
    // of the mechanical stoppers referred to the horizontal position, but in the 
    // case of motor 3 there would be an overflow, so after reaching the mechanical
    // stopper, the motor 3 is moved before setting its position 
    // to avoid the overflow.

    // The other motors are moved from the extremes because with the new motor parameters there is 
    // resonance.

    // A bug on the firmware makes necessary to limit the motors range. Seems that
    // has to be less than 32000


    if (idx == 1) {
	int step = 2000;
	base->GetMOT()->arr[idx].mov(step, true, 30000); //SLEEP(25);
    };	

    if (idx == 2) {
	int step = 3000;
	base->GetMOT()->arr[idx].mov(step, true, 30000); //SLEEP(25);
    };	

    TMotAPS aps = { clb.mcf, degreetoencoder(idx, base->GetEFF()->arr_delta[idx]) };
    if (base->GetMOT()->arr[idx].sendAPS(&aps) != SET_NO_ERR) {
	return CLB_SENDAPS_ERR;
    } SLEEP(100);

    /*-----------------------------------------------------------*/

    map[idx].calib			= true;

    /*-----------------------------------------------------------*/
    return CLB_NO_ERR;
    /*-----------------------------------------------------------*/
}

/*--------------------------------------------------------------------------*/

void CKatana::setMapParam(	long		idx,
				TMapUnit	map_maxpos,
				TMapUnit	map_minpos) {

    map[idx].map_maxpos	= map_maxpos;
    map[idx].map_minpos	= map_minpos;

    map[idx].map_range	= map_maxpos - map_minpos;
	
    map[idx].enc_to_map	= (double)(map[idx].map_range / map[idx].enc_range);
    map[idx].map_to_enc	= (double)(map[idx].enc_range / map[idx].map_range);
}

/*--------------------------------------------------------------------------*/

void CKatana::setTolerance(long idx, TEncUnit enc_tolerance) {
    map[idx].enc_tolerance = enc_tolerance;
}

/*--------------------------------------------------------------------------*/

TEncUnit CKatana::maptoencoder(long idx, TMapUnit value) {

    return (TEncUnit)((value - map[idx].map_minpos) * map[idx].map_to_enc + map[idx].enc_minpos);

}

TEncUnit CKatana::degreetoencoder(long idx, double value) {

    return (TEncUnit)(value * map[idx].deg_to_enc);
}

double CKatana::encodertodegree(long idx, TEncUnit value) {

    return (double)(value * map[idx].enc_to_deg);

}

/*--------------------------------------------------------------------------*/

bool CKatana::checkENLD(long idx, double _degrees) {
	
    return base->GetMOT()->arr[idx].checkENL(this->degreetoencoder(idx,_degrees));
}

/*--------------------------------------------------------------------------*/

bool CKatana::enableCrashLimits() {

	return (base->enableCrashLimits() == SET_NO_ERR);

}

/*--------------------------------------------------------------------------*/

bool CKatana::disableCrashLimits() {

	return (base->disableCrashLimits() == SET_NO_ERR);

}

/*--------------------------------------------------------------------------*/

bool CKatana::unBlock() {

	return (base->unBlock() == SET_NO_ERR);

}

/*--------------------------------------------------------------------------*/

bool CKatana::setCrashLimit(long idx, int limit) {

	return (base->setCrashLimit(idx, limit) == SET_NO_ERR);

}

/*--------------------------------------------------------------------------*/

bool CKatana::recvMPS() {

	return (base->recvMPS() == RECV_NO_ERR);

}

/*--------------------------------------------------------------------------*/

/****************************************************************************/
