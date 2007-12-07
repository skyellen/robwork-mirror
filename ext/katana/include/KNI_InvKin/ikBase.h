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
#ifndef _IKBASE_H_
#define _IKBASE_H_
/******************************************************************************************************************/
#include "KNI/kmlExt.h"
#include "kinematics.h"
/******************************************************************************************************************/

#ifndef TM_ENDLESS
#define TM_ENDLESS -1	 //!< timeout symbol for 'endless' waiting
#endif

enum TRetIKCal {
    IKCal_OUT_OF_RANGE,	 //!< the desired target position is out of range
    IKCal_SUCCESSFUL,	 //!< possible solutions were found
    IKCal_RECVRMP_FAILED //!< receive pos, vel & pwm command failed 
};

enum TRetIKSol {
    IKSol_SOLUTION_NOT_AVAILABLE, //!< the desired target position is out of range
    IKSol_SUCCESSFUL,		  //!< possible solutions were found
};

enum TRetIK {
    IK_MOVING_ERROR,		//!< attempt to move failed
    IK_TARGET_SET,			//!< set target position was successful
    IK_OUT_OF_RANGE,		//!< the desired target position is out of range
    IK_POSITION_REACHED,	//!< desired target position was reached
    IK_RECVRMP_FAILED,		//!< receive pos, vel & pwm command failed
	IK_CRASHED				//!< robot has crashed and the motors are blocked
};

struct  TIKResult {
    double arr_angle[4][6]; //!< 2D array containing the angles of the motors: e.g. if you have 4 possibilities and 6 motors you get this: arr_angle[4][6]; the angles are measured in degrees!
    short  ps;				//!< number of possible solutions that are inside the robot's workspace
    short  ranking[4];	    //!< ranking of distances to source position
    bool   valid[4];	    //!< true if the solution is inside the robot's workspace
};

//------------------------------------------------------------------------------//

class DLLDIR_IK CikBase : public CKatana {

 protected:

    TIKResult	ikr;	

 public:

    CikBase() { ikr.ps=0; };

    /*! \brief Returns the current position of the robot in cartesian units.*/
    void DKApos(double* position);

    /*! \brief Calculates the 4 solutions for the IK problem, indicates which of 
     * them are inside the robot's workspace, and calculates a ranking regarding 
     * the distance of the solutions to the current position.*/
    TRetIKCal IKCalculate(double X, 
			  double Y, 
			  double Z, 
			  double Al, 
			  double Be, 
			  double Ga);

    /*! \brief number of possible solutions that are inside the robot's workspace.*/
    short IKGetPS() { return ikr.ps; };

    /*! \brief Returns an array with the solution number ns solutions are sorted 
     * regarding distance possible values for ns from 1 to 4.*/
    TRetIKSol IKSolution(short ns, 
			 double** arr_angle); 									    
													   
    /*! \brief Moves the robot using 1 of the solutions of IK problem that is 
     * inside the Robot's workspace. */ 
    TRetIK IKMove(short ns, 
		  bool wait = false, 
		  long timeout = TM_ENDLESS);

    /*! \brief Moves the robot using IK. */
    TRetIK IKGoto(double X, 
		  double Y, 
		  double Z, 
		  double Al, 
		  double Be, 
		  double Ga, 
		  bool wait = false, 
		  long timeout = TM_ENDLESS);	


};

/******************************************************************************************************************/
#endif //_IKBASE_H_
/******************************************************************************************************************/
