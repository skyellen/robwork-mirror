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
#ifndef _KMLEXT_H_
#define _KMLEXT_H_
/******************************************************************************************************************/
#include "kmlBase.h"
#include "dllexport.h"
/******************************************************************************************************************/

/*----------------------------------------------------------------------------------------------------------------*/

enum TSearchDir {	//!< search direction for the meachanical stopper
	DIR_POSITIVE,
	DIR_NEGATIVE
};

enum TMapFunc {	//!< different measure functions for mapping system
	MF_ENCODE,	//!< e.g. -32768 to +32767
	MF_RADIAN,	//!< e.g. -PI to +PI
	MF_LINEAR	//!< e.g. -1.0 to +1.0
};

/*----------------------------------------------------------------------------------------------------------------*/

#define TEncUnit int	//!< internal mapping system unit
#define TMapUnit double		//!< external mapping system uint

/*----------------------------------------------------------------------------------------------------------------*/

enum TRetCLB {
	CLB_NO_ERR,				//!< calibration was successful
	CLB_DISABLED,			//!< calibration was disabled
	CLB_SEARCHMECHSTOP_ERR,	//!< could not find mechanical stopper
	CLB_SENDAPS_ERR,		//!< could not send target position command
	CLB_SENDTPS_ERR,		//!< could not send actual position command
	CLB_SENDSCP_ERR,		//!< could not send static controller command
	CLB_SENDDYL_ERR			//!< could not send dynamic limits command
};

/*----------------------------------------------------------------------------------------------------------------*/

/*!	\brief	Mapping structure
 */
struct TMotMAP {
	TEncUnit	enc_range;		//!< encoder range between the mech. stoppers
	TMapUnit	map_range;		//!< mapping range between the mech. stoppers

	TEncUnit	enc_maxpos;		//!< minimum available encoder position
	TEncUnit	enc_minpos;		//!< maximum available encoder position

	TMapUnit	map_maxpos;		//!< maximum available mapping position
	TMapUnit	map_minpos;		//!< minimum available mapping position

	double		enc_to_map;		//!< transform quotient: encoder -> map
	double		map_to_enc;		//!< transform quotient: map -> encoder
	double		enc_to_deg;		//!< transform quotient: encoder -> degree
	double		deg_to_enc;		//!< transform quotient: degree -> encoder

	TEncUnit	enc_tolerance;	//!< allowed tolerance/precision of moves
	bool		calib;			//!< true if calibrated else false
};

/*----------------------------------------------------------------------------------------------------------------*/

/*!	\brief	Calibration structure for single motors.
 */
struct TMotCLB {
	bool		enable;			//!< enable/disable

	TSearchDir	dir;			//!< search direction for mech. stopper
	TEncUnit	diff;			//!< step size during the search
	TMotCmdFlg	mcf;			//!< motor flag after calibration

	TEncUnit	enc_range;		//!< range between the mech. stoppers
	TMapUnit	map_maxpos;		//!< maxpos - mapping
	TMapUnit	map_minpos;		//!< minpos - mapping
	long		enc_per_circle;	//!< count of encoder units per 360 degree circle
	TEncUnit	enc_tolerance;	//!< encoder units of tolerance to accept that a position has been reached

	long		timeout;		//!< max. calibration time
};

/*!	\brief Calibration structure for the whole robot.
 */
struct TKatCLB {
	long		cnt;	//!< count of motors to calibrate
	TMotCLB*	clb;	//!< array of calibration struct's
	TMotSCP*	scp;	//!< array of static controller parameters
	TMotDYL*	dyl;	//!< array of dynamic limits
};

/*----------------------------------------------------------------------------------------------------------------*/


/*!	\brief	Extended Katana class with additional functions
 *
 *	This class uses the 'CKatBase* base' object to refer to a Katana robot;
 *	but most important benefit using this class is that it can read a
 *	configuration file to construct a mapping structure to control to Katana
 *	in other than encoder units.
 */

class DLLDIR CKatana {
protected:
	//-------------------------------------//
	CKatBase*	base;	//!< base katana
	TMotMAP*	map;	//!< mapping struct

public:
	//-------------------------------------//
	CKatBase* GetBase()        { return base;     } //!< Returns pointer to 'CKatBase*'

	TMotMAP   GetMAP(int idx)  { return map[idx]; } //!< Returns mapping struct to give index

protected:
	//------------------------------------------------------------------------------//

	void default_map(TMotMAP* map);	//!< inits the mapping array to default values

	//------------------------------------------------------------------------------//

	/*!	\brief	Sets the tolerance range in encoder units for the robots movements.
	 */
	void setTolerance(long idx, TEncUnit enc_tolerance);

	//------------------------------------------------------------------------------//

public:
	//------------------------------------------------------------------------------//
	/*!	\brief Constructor

	 */
	CKatana()  { base = new CKatBase; map = 0L;	}
	/*!	\brief Destructor
	 */
	~CKatana() { delete base; delete map;		}
	//------------------------------------------------------------------------------//
	/*! \brief Create routine
	 */
	bool create(const char* cfgFile,		//!< configuration file
				CCplBase* _protocol			//!< protocol to be used
				);

	/*! \brief Create routine
	 */
	bool create(TKatGNL& gnl,				//!< katana initial attributes
				TKatMOT& mot,				//!< motor initial attributes
				TKatSCT& sct,				//!< sensor controller initial attributes
				TKatEFF& eff,				//!< end effector initial attributes
				CCplBase* _protocol			//!< protocol to be used
				);
	//------------------------------------------------------------------------------//

	bool	calibrate(TKatCLB*		clb			//!< calibration struct for all motors
						);

	bool	calibrate(const char*	cfgFile		//!< configuration file
						);

	TRetCLB calibrate(long			idx,		//!< motor index
					  TMotCLB		clb,		//!< calibration struct for one motor
					  TMotSCP		scp,		//!< static controller parameters
					  TMotDYL		dyl			//!< dynamic controller parameters
						);

	//------------------------------------------------------------------------------//

	bool searchMechStop(long		idx,					//!< motor index
						TSearchDir	dir,					//!< search direction
						TEncUnit	dif,					//!< step size
						long		timeout,				//!< max. search time
						TMotSCP		scp,					//!< static controller parameters
						TMotDYL		dyl						//!< dynamic controller parameters
						);

	void setMapParam(long		idx,			//!< motor index
					TMapUnit	map_maxpos,		//!< maximum allowed mapping position
					TMapUnit	map_minpos		//!< minimum allowed mapping position
					);

	//------------------------------------------------------------------------------//
	/*!	\brief	Increments the motor specified by an index postion in mapping units.
	 */
	TRetMOV inc(long idx, TMapUnit dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Decrements the motor specified by an index postion in mapping units.
	 */
	TRetMOV dec(long idx, TMapUnit dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Moves the motor specified by an index to a given target position
	 *			in mapping units.
	 */
	TRetMOV mov(long idx, TMapUnit tar, bool wait = false, long timeout = TM_ENDLESS);

	//------------------------------------------------------------------------------//
	/*!	\brief	Increments the motor specified by an index postion in degree units.
	 */
	TRetMOV incDegrees(long idx, double dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Decrements the motor specified by an index postion in degree units.
	 */
	TRetMOV decDegrees(long idx, double dif, bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Moves the motor specified by an index to a given target position
	 *			in degree units.
	 */
	TRetMOV movDegrees(long idx, double tar, bool wait = false, long timeout = TM_ENDLESS);

	//------------------------------------------------------------------------------//


	/*!	\brief	Sets the target position of a motor in Map Units and allows the movement 
	 *			of that motor during the parallel movement.
	 */
	void setTPSP(long idx, TMapUnit _tar);
	/*!	\brief	Forbid the movement of all the motors during the parallel movement.
	 */
	void resetTPSP();
	/*!	\brief	Moves the allowed motors simultaneously.
	 */
	TRetMOV sendTPSP(bool wait = false, long timeout = TM_ENDLESS);
	/*!	\brief	Sets the target position of a motor in degree Units and allows the movement 
	 *			of that motor during the parallel movement.
	 */
	void setTPSPDegrees(long idx, double _tar);

	//------------------------------------------------------------------------------//
	// public just for dubbuging purposes
	/*!	\brief	Check if the absolute position in degrees is out of range.
	 */

	bool checkENLD(long idx, double _degrees);

	/*!	\brief	Converts a value from Map units to Encoder units.
	 */
	TEncUnit maptoencoder(long idx, TMapUnit value);

	/*!	\brief	Converts a value from Degree units to Encoder units.
	 */
	TEncUnit degreetoencoder(long idx, double value);

	/*!	\brief	Converts a value from Encoder units to Degree units.
	 */
	double encodertodegree(long idx, TEncUnit value);

	//------------------------------------------------------------------------------//

	/*!\brief crash limits enable
	*/	
	bool enableCrashLimits();
	/*!\brief crash limits disable
	*/	
	bool disableCrashLimits();
	/*!\brief unblock robot after a crash
	*/	
	bool unBlock();
	/*!\brief unblock robot after a crash
	*/	
	bool setCrashLimit(long idx, int limit);

	//------------------------------------------------------------------------------//
	/*!\brief receive all motor positions simultaneously
	*/	
	bool recvMPS(); 
	/*!\brief get the position of a motor in encoder units
	*/	
	short GetMPS(long idx) { return base->GetMPS(idx); }

	//------------------------------------------------------------------------------//

};

/******************************************************************************************************************/
#endif //_KMLEXT_H_
/******************************************************************************************************************/
