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

/********************************************************************************/
#ifndef _LMBASE_H_
#define _LMBASE_H_
/********************************************************************************/
#include "KNI_InvKin/ikBase.h"
#include <math.h>
/********************************************************************************/

enum TRetLM {
    LM_COMMUNICATION_ERROR, //!< error sending L command
    LM_MOVING_ERROR,	    //!< attempt to move failed
    LM_TARGET_SET,	    //!< set target position was successful
    LM_OUT_OF_RANGE,	    //!< the desired target position is out of range
    LM_KINEMATICS_ERROR,    //!< the desired target position is out of range
    LM_POSITION_REACHED,    //!< desired target position was reached
	LM_CRASHED				//!< robot has crashed and the motors are blocked
};

enum TRetLMFill {
    LMFill_KINEMATICS_ERROR, //!< 
    LMFill_OUT_OF_RANGE,     //!< 
    LMFill_SUCCESSFUL	     //!< 
};

enum TRetLMCalcP {
    LMCalcP_KINEMATICS_ERROR, //!< 
    LMCalcP_OUT_OF_RANGE,     //!<
    LMCalcP_SUCCESSFUL	      //!<
};

//------------------------------------------------------------------------------//

/*!	\brief	[LM] linear movement: points to be interpolated
 */
struct TLM_points {
    double pos;  //!< position of one point to be interpolated (% refer to the total trajectory)
    double time; //!< time that it takes to reach the point (from starting position)
};

/*!	\brief	[LM] linear movement: parameters
 */
struct TLMtrajectory {
    double*	arr_actpos;	  //!< current position in cartesian units
    double*	arr_tarpos;	  //!< target position in cartesian units
    int		distance;	  //!< distance between target and current position
    double	time;		  //!< time that it takes from current position to target position
    double	dt;		  //!< time elapsed between one step and the next one
    short	number_of_points; //!< number of points to interpolate
    TLM_points*	points;		  //!< points to be interpolated
    short**	motors;		  //!< motor position in each point to be interpolated
    double**	derivatives;	  //!< second order derivatives of the polinomes that join the points, in the points
    double***	coefficients;	  //!< coefficients of the polinomes that join the points
    short***	parameters;	  //!< parameters to be sent in the command 'L' packet
};

/*!	\brief	[LM] Store intermediate targets for multiple linear movements
 */
struct TMLMIP {
    short	mlm_intermediate_pos[5];	  //!< current position in cartesian units
};

//------------------------------------------------------------------------------//

class DLLDIR_LM CLMBase : public CikBase {

    TLMtrajectory trajectory;

 private:
    //calcs the current pos of the tool in cartesian coordinates
    void actualPOS(double* arr_pos);

    TRetLMFill fillPoints(double vmax);
    void polDeviratives();
    void polCoefficients();

    TRetLMCalcP calcParameters(double* arr_actpos, 
			       double* arr_tarpos, 
			       double vmax);

 public:
    void initLM();
    TRetLM movLM(double X, double Y, double Z, 
		 double Al, double Be, double Ga,      
		 bool exactflag, double vmax, bool wait=false, long timeout = TM_ENDLESS);	

    TRetLM movLM2P(double X1, double Y1, double Z1, 
		 double Al1, double Be1, double Ga1,      
				  double X2, double Y2, double Z2, 
		 double Al2, double Be2, double Ga2, 
				  bool exactflag, double vmax, bool wait=false, long timeout = TM_ENDLESS);	

};

/********************************************************************************/
#endif //_IKBASE_H_
/********************************************************************************/
