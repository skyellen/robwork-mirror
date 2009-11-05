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


/*! \mainpage "Katana Native Interface Documentation"
 *
 * \section	intro	I. Introduction
 *
 * The Katana Native  Interface [KNI] is a software library, which enables you to write controlling software for
 * Katana robots produced by Neuronics. The KNI is written in the C++ programming language - this means that the
 * library is organized in classes and structures. You should be familiar with object oriented thinking and C++
 * programming language to understand this library.
 * The library could also have been implemented in C or another native language without objects, because the
 * complexity of the software does not demand a solution based on OOP (object oriented programming) at the moment,
 * but for future plans and extensions on the Katana robot we have decided to use OOP. If you have feedback about
 * the organisation of the library please write an email to softdev@neuronics.ch. Please report also bugs you will
 * eventually detect, so we can improve the KCI and this documentation.
 *
 * \section structview	II. Structure Overview
 *
 * The library uses three layers:
 *
 * The Communication Device Layer [CDL] is responsible for devices such as serial port, universal serial bus and
 * others. Actually this layer supports only the serial port device. It’s encapsulated into the CCdlCOM class.
 *
 * The Communication Protocol Layer [CPL] defines the language (protocol), which the robot and the KNI use to
 * communicate. At the moment there are two protocols: Serial-Zero Protocol and Serial-CRC Protocol. They are
 * encapsulated into the CCplSerialZero and CCplSerialCRC classes.
 *

 * The Katana Robot Model Layer [KML] defines a model of the robot in the software. It supports methods (functions)
 * to update the model from the real robot and vice versa. The model is encapsulated into the CKatana, CKatBase,
 * CMotBase, CSctBase classes.
 */

/****************************************************************************/
#ifndef _KMLBASE_H_
#define _KMLBASE_H_
/****************************************************************************/
#include "cplBase.h"
#include "dllexport.h"
/****************************************************************************/

#if !defined (BYTE_DECLARED)
#define BYTE_DECLARED
typedef unsigned char byte;	//!< type specification (8 bit)
#endif

//--------------------------------------------------------------------------//

#define TEncUnit int	//!< internal mapping system unit
#define TM_ENDLESS -1	//!< timeout symbol for 'endless' waiting

//--------------------------------------------------------------------------//

class CKatBase;	//katana
class CMotBase;	//motor
class CSctBase;	//sensor contoller

//--------------------------------------------------------------------------//

/*!	\brief command return values
 */
typedef enum  TRetCMD {
	NO_ERR,	//!< communication was successfull
	RTOERR,	//!< a timeout error occured
	CRCERR,	//!< a CRC-checking error occured
	SLVERR,	//!< an error in the internal robot bus occured
	BADASW	//!< a not expected answer (only CKatBase::recvECH)
};


/*!	\brief	Communication return values
 */
enum TRetSetP {
	SET_WRONG_PARAMETERS,	//!< Parameters are set but wrong
	SET_WRITTING_ERR,		//!< Error writting parameters
	SET_READING_ERR,		//!< Error reading parameters
	SET_NO_ERR				//!< No error
};

enum TRetRecvP {
	RECV_READING_ERR,		//!< Error reading parameters
	RECV_NO_ERR				//!< No error
};

enum TRetMOV {
	MOV_TARGET_SET,			//!< set target position was successful
	MOV_POSITION_REACHED,	//!< desired target position is reached
	MOV_RANGE_ERROR,		//!< desired position is out of range
	MOV_TIMEOUT,			//!< a timout evet occured
	MOV_SENDTPS_FAILED,		//!< send target position command failed
	MOV_RECVRMP_FAILED,		//!< receive pos, vel & pwm command failed
	MOV_CRASHED				//!< crash has ocurr
};

/****************************************************************************/
// CKatBase ----------------------------------------------------------------//
/****************************************************************************/

/*!	\brief	[GNL] general robot attributes
 */
struct  TKatGNL {
	byte		adr;			//!< jumper adress
};

/*!	\brief	[MFW] master firmware version/revision number
 */
struct  TKatMFW {
	byte		ver;			//!< version
	byte		rev;			//!< revision
};

/*!	\brief	[IDS] identification string
 */
struct  TKatIDS {
	byte		strID[256];		//!< id string
};

/*!	\brief	[CTB] command table defined in the firmware
 */
struct  TKatCTB {
	byte		cmdtbl[256];	//!< command table
};

/*!	\brief	[CBX] connector box
 */
struct TKatCBX {
	bool inp[2];				//!< input: green & red LED
	bool out[2];				//!< output: green & red LED
};

/*!	\brief	[ECH] echo
 */
struct  TKatECH {
	byte		echo;			//!< echo answer
};

/*! \brief Inverse Kinematics structure of the endeffektor
 *
 *	This structure describes the properties of the endeffector and it's used for the inverse
 *	kinematic calculations. An endeffector is a point where the attributes of this structure
 *	belong to. Please remember that the actual inverse kinematic calculations have been set
 *	up <b>only</b> for the Katana <b>6M</b> robot! So do not be astonished if you get strange
 *	behaviour with a Katana <b>5M</b>.
 */

struct TKatEFF {
	double		arr_segment[4];		//!< length of the Katana segments
	double		arr_position[3];	//!< position of the endeffector in space
	double		arr_orientation[3];	//!< orientation of the endeffector in space

	double		arr_angle[4][6];	//!< 2D array containing the angles of the motors: e.g. if you have 4 possibilities and 6 motors you get this: arr_angle[4][6]; the angles are measured in degrees!
	double		arr_delta[6];		//!< Inital segment angle positions 
};


//--------------------------------------------------------------------------//

/*!	\brief	motor description (partly)
 */
struct  TMotDesc {
	byte		slvID;			//!< slave number
};

/*!	\brief	[MOT] every motor's attributes
 */
struct  TKatMOT {
	short		cnt;			//!< count of motors
	CMotBase*	arr;			//!< array of motors
	TMotDesc*	desc;			//!< description[]
};

//--------------------------------------------------------------------------//

/*!	\brief sensor controller description (partly)
 */
struct  TSctDesc {
	byte		ctrlID;			//!< controller number (ID)
	short		sens_res;		//!< resolution: 8/12 bit
	short		sens_count;		//!< count of sensors
};

/*!	\brief	[SCT] every sens ctrl's attributes
 */
struct  TKatSCT {
	short		cnt;			//!< count of sens ctrl's
	CSctBase*	arr;			//!< array of sens ctrl's
	TSctDesc*	desc;			//!< description[]
};

/*!	\brief	[MPS] motor positions when they are read simultaneously
 */
struct  TKatMPS {
	short*		pos;			//!< store motor positions
};

//--------------------------------------------------------------------------//

/*!	\brief	Base Katana class
 *
 *	This class is the main object controlling the whole katana; to use it, it
 *	has to be initilized by using it's init function; those function expects
 *	a initilized protocol class, which in turn expects an initilized device!
 *	after the initialization, it does not mean that the coordinates (encoder
 *	values) of the motors have been set correctly; for that a calibration is
 *	needen; that calibration can be executed either by using the CKatana class
 *	in the 'kmlExt' module (which encapsulates this class) or by writing your
 *	own calibrations function..
 */
class  DLLDIR CKatBase {

protected:
	TKatGNL gnl;	//!< katana general
	TKatMFW mfw;	//!< master's firmware version/revision
	TKatIDS ids;	//!< ID string
	TKatCTB ctb;	//!< cmd table
	TKatCBX cbx;	//!< connector box
	TKatECH ech;	//!< echo

	TKatMOT mot;	//!< motors
	TKatSCT sct;	//!< sensor controllers
	TKatEFF	eff;	//!< end effector
	TKatMPS	mps;	//¦< motor positions when they are read simultaneoulsy

public:
	/*! \brief Get a pointer to the desired structure
	*/	const TKatGNL* GetGNL() { return &gnl; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatMFW* GetMFW() { return &mfw; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatIDS* GetIDS() { return &ids; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatCTB* GetCTB() { return &ctb; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatCBX* GetCBX() { return &cbx; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatECH* GetECH() { return &ech; }

	/*! \brief Get a pointer to the desired structure
	*/	const TKatMOT* GetMOT() { return &mot; }
	/*! \brief Get a pointer to the desired structure
	*/	const TKatSCT* GetSCT() { return &sct; }
	/*! \brief Get motor 1 position after reading all motor positions simultaneously
	*/	short GetMPS(long idx) { return mps.pos[idx]; }
	/*! \brief Get a pointer to the desired structure
	*/	TKatEFF* GetEFF() { return &eff; }


protected:
	CCplBase* protocol;	//!< protocol interface

public:
	/*!	\brief	destructor
	*/	virtual ~CKatBase() { free(); }

	virtual bool init(
		const TKatGNL _gnl,		//!< general attributes
		const TKatMOT _mot,		//!< motor attributes
		const TKatSCT _sct,		//!< sensor controller attributes
		const TKatEFF _eff,		//!< end effector attributes
		CCplBase* _protocol		//!< desired protocol
		);

	/*! \brief	frees allocated resources
	*/	virtual void free();

	/*!\brief receive data
	*/	TRetRecvP	recvMFW();
	/*!\brief receive data
	*/	TRetRecvP	recvIDS();
	/*!\brief receive data
	*/	TRetRecvP	recvCTB();
	/*!\brief receive data
	*/	TRetRecvP	recvGMS();
	/*!\brief receive data
	*/	TRetRecvP	recvCBX();
	/*!\brief receive data
	*/	TRetRecvP	recvECH();
	/*!\brief receive data
	*/	TRetRecvP	recvNMP();
	/*!\brief read all motor positions simultaneously
	*/	TRetRecvP	recvMPS();

	/*!\brief send data
	*/	TRetSetP sendCBX(const TKatCBX* _cbx);

	/*!\brief parallel movements
	*/	TRetSetP sendTPSP();

	/*!\brief crash limits enable
	*/	TRetSetP enableCrashLimits();
	/*!\brief crash limits disable
	*/	TRetSetP disableCrashLimits();
	/*!\brief unblock robot after a crash
	*/	TRetSetP unBlock();
	/*!\brief unblock robot after a crash
	*/	TRetSetP setCrashLimit(long idx, int limit);

	/*!\brief linear movements
	*/	TRetSetP sendSLMP(byte* p);

	/*!\brief linear movements
	*/	TRetSetP sendSLM(bool exactflag);

};

/****************************************************************************/
// CMotBase ----------------------------------------------------------------//
/****************************************************************************/

//--------------------------------------------------------------------------//

/*!	\brief	command flags
 */
typedef enum TMotCmdFlg {
	MCF_OFF		= 0,		//!< set the motor off
	MCF_FREEZE	= 8,		//!< freeze the motor
	MCF_ON		= 24,		//!< set the motor on
};

/*!	\brief	status flags
 */
typedef enum TMotStsFlg	{
	MSF_MECHSTOP	= 1,	//!< mechanical stopper reached
	MSF_MAXPOS	= 2,		//!< max. position was reached
	MSF_MINPOS	= 4,		//!< min. position was reached
	MSF_DESPOS	= 8,		//!< in desired position
	MSF_NORMOPSTAT	= 16,	//!< trying to follow target
	MSF_NOTVALID	= 128	//!< motor data not valid
};

//--------------------------------------------------------------------------//

/*!	\brief	[GNL] motor generals
 */
struct TMotGNL {
	CKatBase*	own;		//!< parent robot
	byte		SID;		//!< slave ID
};

/*!	\brief	[SFW] slave firmware
 */
struct TMotSFW {
	byte		version;	//!< firmware version number
	byte		subversion;	//!< firmware subversion number
	byte		revision;	//!< firmware revision number
	byte		type;		//!< firmware type
	byte		subtype;	//!< firmware subtype
};

/*!	\brief	[APS] actual position
 */
struct TMotAPS {
	TMotCmdFlg	mcfAPS;		//!< motor command flag
	short		actpos;		//!< actual position
};

/*!	\brief	[TPS] target position
 */
struct TMotTPS {
	TMotCmdFlg	mcfTPS;		//!< motor command flag
	short		tarpos;		//!< target position
};

/*!	\brief	[SCP] static controller parameters
 */
struct TMotSCP {

	//--------------- Motor old parameters -------------------------------//
	//
	byte		maxppwm;	//!< max. val for pos. voltage
	byte		maxnpwm;	//!< max. val for neg. voltage; pos!
	byte		kP;			//!< prop. factor of pos comp
	byte		kI;			//!< not yet active
	byte		kD;			//!< derivate factor of pos comp
	byte		kARW;		//!< not yet active
	//byte		kSpeed;		//!< prop. factor of speed limit comp
	byte		kP_speed;	//!< Proportional factor of the speed compensator
	byte		kI_speed;	//!< Integral factor of the speed compensator
	byte		kD_speed;	//!< Derivative factor of the speed compensator

	//--------------- Motor new parameters -------------------------------//
	//
	byte		maxppwm_nmp;		//!< Max. value for positive voltage (0 => 0%, +70 => 100%)
	byte		maxnpwm_nmp;		//!< Max. value for negative voltage (0 => 0%, +70 => 100%)
	byte		kspeed_nmp;				//!< Proportional factor of speed compensator
	byte		kpos_nmp;				//!< Proportional factor of position compensator
	byte		kI_nmp;					//!< Integral factor (1/kI) of control output added to the final control output
	TEncUnit	crash_limit_nmp;		//!< Limit of error in position
	TEncUnit	crash_limit_lin_nmp;	//!< Limit of error in position in linear movement
};

/*!	\brief	[DYL] dynamic limits
 */
struct TMotDYL {

	//--------------- Motor old parameters -------------------------------//
	//
	byte		maxaccel;	//!< max acceleration
	byte		maxdecel;	//!< max deceleration
	short		minpos;		//!< not yet active
	short		maxpspeed;	//!< max. allowed forward speed
	short		maxnspeed;	//!< max. allowed reverse speed; pos!
	//byte		maxpcurr;	// no more active
	//byte		maxncurr;	// no more active
	byte		maxcurr;	//!< max current
	byte		actcurr;	//!< actual current

	//--------------- Motor new parameters -------------------------------//
	//
	byte		maxaccel_nmp;		//!< Maximal acceleration and deceleration
	short		maxpspeed_nmp;		//!< Max. allowed forward speed
	short		maxnspeed_nmp;		//!< Max. allowed reverse speed
	byte		maxcurr_nmp;		//!< set the maximal current
};

/*!	\brief	[PVP] position, velocity, pulse width modulation
 */
struct TMotPVP {
	TMotStsFlg	msf;		//!< motor status flag
	short		pos;		//!< position
	short		vel;		//!< velocity
	byte		pwm;		//!< pulse with modulation
};

/*!	\brief	[ENL] limits in encoder values
 */
struct TMotENL {
	TEncUnit	enc_range;		//!< motor's range in encoder values
	TEncUnit	enc_minpos;		//!< motor's minimun position in encoder values 
	TEncUnit	enc_maxpos;		//!< motor's maximun position in encoder values
	TEncUnit	enc_per_cicle;	//!< number of encoder units needed to complete 360 degrees;
	TEncUnit	enc_tolerance;	//!< encoder units of tolerance to accept that a position has been reached
};

//--------------------------------------------------------------------------//

/*!	\brief	Motor class
 *

 *	This class allows to control one motor; to control a motor it has to be
 *	initialized by using the init function. And the usage the internal allocated
 *	resources should be deallocated by using the 'free' method.

 */
class DLLDIR CMotBase {

	friend class CKatBase;

protected:
	TMotGNL gnl;	//!< motor generals
	TMotAPS aps;	//!< actual position
	TMotTPS tps;	//!< target position
	TMotSCP scp;	//!< static controller parameters
	TMotDYL dyl;	//!< dynamic limits
	TMotPVP pvp;	//!< reading motor parameters
	TMotSFW sfw;	//!< slave firmware
	TMotENL enl;	//!< motor limits in encoder values
	bool	freedom;	//!< if it is set, it will move on a parallel movement
	bool	nmp;		//!< true if new motor parameters are implemented on the firmware
	bool	blocked;	//¦< true if the motor was blocked due to a crash of the robot

public:
	/*! \brief Get a pointer to the desired structure
	*/const TMotGNL* GetGNL() { return &gnl; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotAPS* GetAPS() { return &aps; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotTPS* GetTPS() { return &tps; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotSCP* GetSCP() { return &scp; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotDYL* GetDYL() { return &dyl; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotPVP* GetPVP() { return &pvp; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotSFW* GetSFW() { return &sfw; }
	/*! \brief Get a pointer to the desired structure
	*/const TMotENL* GetENL() { return &enl; }
	/*! \brief Get the value of the freedom property
	*/const bool GetFreedom() { return freedom; }
	/*! \brief Get the value of the blocked property
	*/const bool GetBlocked() { return blocked; }

protected:
	CCplBase* protocol;	//!< protocol interface

public:
	virtual ~CMotBase() { free(); }	//destructor

	bool init(CKatBase* _own, const TMotDesc _motDesc, CCplBase* protocol);
	void free();

	/*!\brief send data
	*/	TRetSetP sendAPS(const TMotAPS* _aps);
	/*!\brief send data
	*/	TRetSetP sendTPS(const TMotTPS* _tps);
	/*!\brief send data
	*/	TRetSetP sendSCP(const TMotSCP* _scp);
	/*!\brief send data
	*/	TRetSetP sendDYL(const TMotDYL* _dyl);

	/*!\brief receive data
	*/	TRetRecvP recvPVP();
	/*!\brief receive data
	*/	TRetRecvP recvSCP();
	/*!\brief receive data
	*/	TRetRecvP recvDYL();
	/*!\brief receive data
	*/	TRetRecvP recvSFW();


	/*!\brief parallel movement
	*/	void setTPSP(int _tar);
	/*!\brief parallel movement
	*/	void resetTPSP();

	/*!\brief set limits in encoder values
	*/	bool setENL(TEncUnit _enc_range, 
					TEncUnit _enc_minpos, 
					TEncUnit _enc_maxpos, 
					TEncUnit _enc_per_cicle,
					TEncUnit _enc_tolerance);		

	/*!\brief check limits in encoder values
	*/	bool checkENL(TEncUnit _enc_value);	

	/*!	\brief	Increments the motor specified by an index postion in encoder units.
	 */
	TRetMOV inc(TEncUnit dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Decrements the motor specified by an index postion in encoder units.
	 */
	TRetMOV dec(TEncUnit dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Moves the motor specified by an index to a given target position in encoder units.
	 */
	TRetMOV mov(TEncUnit tar, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	unblock the motor.
	 */
	TRetSetP resetBlocked();

};


/****************************************************************************/
// CSctBase ----------------------------------------------------------------//
/****************************************************************************/

/*!	\brief	[GNL] controller generals
 */
struct TSctGNL {
	CKatBase*	own;			//!< parent robot
	byte		SID;			//!< slave ID
	short		res;			//!< resolution: 8/12 bit
};

/*!	\brief	[DAT] sensor data
 */
struct  TSctDAT {
	short		cnt;			//!< count of sensors
	short*		arr;			//!< sensor data
};

//--------------------------------------------------------------------------//

/*!	\brief	Sensor Controller class
 *
 *	By using this class you can get access to the sensor data; to do so you
 *	should (after initialization) call 'recvDat()' to updated the internal
 *	'TSctDAT dat' structure; after the updated you can read out the values
 *	by using the 'GetDAT()' function, which will return a constant pointer
 *	to the internal 'dat' structure.
 */
class DLLDIR CSctBase {

	friend class CKatBase;

protected:
	TSctGNL	gnl;	//!< controller generals
	TSctDAT dat;	//!< sensor data

public:
	const TSctGNL* GetGNL() { return &gnl; }
	const TSctDAT* GetDAT() { return &dat; }

protected:
	CCplBase* protocol;	//!< protocol interface

public:
	virtual ~CSctBase() { free(); }	//destructor

	bool init(CKatBase* _own, const TSctDesc _sctDesc, CCplBase* protocol);
	void free();

	/*!\brief receive data
	*/	TRetCMD recvDAT();
};

/****************************************************************************/
#endif //_KMLBASE_H_
/****************************************************************************/
