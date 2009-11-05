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
#include "KNI_LM/lmBase.h"
#include "KNI/functions.h"
#include <iostream>	    //for messages: printf()
/****************************************************************************/


void CLMBase::actualPOS(double* arr_pos) {

    TKatEFF* eff = base->GetEFF();
    // stores the length of the links

    double arr_del[6];
    // stores the current position of each motor in degrees

    for (int i=0; i<base->GetMOT()->cnt; i++) {
	base->GetMOT()->arr[i].recvPVP();			
	arr_del[i] = (double)base->GetMOT()->arr[i].GetPVP()->pos * 
	    GetMAP(i).enc_to_deg;
    } KATANA_VWL(eff->arr_segment,arr_pos,arr_del); 
}

/****************************************************************************/

TRetLMFill CLMBase::fillPoints(double vmax) {

    double acc = 1500;	// currently fixed
    double dec = 1500;	// currently fixed

    trajectory.distance = 
	(int) pow(pow((trajectory.arr_actpos[0] 
		       - trajectory.arr_tarpos[0]),2) 
		  + pow((trajectory.arr_actpos[1] 
			 - trajectory.arr_tarpos[1]),2) 
		  + pow((trajectory.arr_actpos[2] 
			 - trajectory.arr_tarpos[2]),2),
		  0.5);

    // minimun distance in order to reach vmax
    double dgrez = (pow(vmax,2)/2) * ((1/acc)+(1/dec));	
    // if distance < dgrez -> vmax is never reached

    for (int i=0; i<trajectory.number_of_points; i++) {

	//------------------------------------------------------------------//
	// Fill trajectory.points with the relative positition and absolute 
	// time at each point to be interpolated 

	if (trajectory.distance > dgrez) { //vmax will be reached

	    trajectory.time = 
		(trajectory.distance/vmax) 
		+ ((vmax/2)*((1/acc)+(1/dec)));
	    trajectory.points[i].time = 
		((double)i/(trajectory.number_of_points-1)) 
		* trajectory.time;

            // acceleration phase
	    if (trajectory.points[i].time < (vmax/acc)) { 
		trajectory.points[i].pos = 
		    (acc / 2) 
		    * pow(trajectory.points[i].time,2);
	    } else {
		if (trajectory.points[i].time 
		    < (trajectory.time - (vmax/dec))) {	// maximum speed
		    trajectory.points[i].pos = 
			vmax 
			* (trajectory.points[i].time 
			   - ((1/2) 
			      * vmax/acc));
		} else { // deceleration phase
		    trajectory.points[i].pos = 
			trajectory.distance 
			- (dec*((pow(trajectory.points[i].time,2)/2)
				-(trajectory.time*trajectory.points[i].time)
				+(pow(trajectory.time,2)/2)));
		}
	    }

	} else { // vmax will not be reached 

	    trajectory.time = (8 * trajectory.distance) / pow((acc + dec),
							      (1/2));
	    trajectory.points[i].time = 
		((double)i/trajectory.number_of_points) 
		* trajectory.time;
	    if (trajectory.points[i].time < (trajectory.time/2)) {
		trajectory.points[i].pos = 
		    (acc/2) 
		    * pow(trajectory.points[i].time,2);
	    } else {
		trajectory.points[i].pos = 
		    trajectory.distance 
		    - (dec*((pow(trajectory.points[i].time,2)/2)
			    -(trajectory.time*trajectory.points[i].time)
			    +(pow(trajectory.time,2)/2)));
	    }

	}

	//------------------------------------------------------------------//
	// Fill trajectory.dt, it will be used in the following calculations
	trajectory.dt = 
	    trajectory.time / (trajectory.number_of_points - 1); 
	//------------------------------------------------------------------//
	// Fill trajectory.motors with the position of the motors at each 
	// point to be interpolated

	double Xm = trajectory.arr_actpos[0] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[0] - trajectory.arr_actpos[0]));
	double Ym = trajectory.arr_actpos[1] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[1] - trajectory.arr_actpos[1]));
	double Zm = trajectory.arr_actpos[2] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[2] - trajectory.arr_actpos[2]));
	double Alm = trajectory.arr_actpos[3] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[3] - trajectory.arr_actpos[3]));
	double Bem = trajectory.arr_actpos[4] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[4] - trajectory.arr_actpos[4]));
	double Gam = trajectory.arr_actpos[5] 
	    + ((trajectory.points[i].pos/trajectory.distance) 
	       * (trajectory.arr_tarpos[5] - trajectory.arr_actpos[5]));

	if (IKCalculate(Xm, Ym, Zm, Alm, Bem, Gam) != IKCal_SUCCESSFUL)
	    return LMFill_KINEMATICS_ERROR;

	double* arr_angle = new double[6];
	if (IKSolution(1,&arr_angle) != IKSol_SUCCESSFUL)
	    return LMFill_KINEMATICS_ERROR;

	for (int j=0; j<base->GetMOT()->cnt; j++) 
	    trajectory.motors[j][i] = degreetoencoder(j,arr_angle[j]);
    } return LMFill_SUCCESSFUL;
}

/****************************************************************************/

void CLMBase::polDeviratives() {

    // These calculations are fixed to 5 points and 5 motors,
    //should be extended to n elements...

    double e1,e2,e3;	
    for (int mot=0; mot<base->GetMOT()->cnt; mot++) {

	e1 = (double)(3/trajectory.dt) 
	    * (double)(trajectory.motors[mot][2] 
		       - trajectory.motors[mot][0]);
	e2 = (double)(3/trajectory.dt) 
	    * (double)(trajectory.motors[mot][3] 
		       - trajectory.motors[mot][1]);
	e3 = (double)(3/trajectory.dt) 
	    * (double)(trajectory.motors[mot][4] 
		       - trajectory.motors[mot][2]);
		
	trajectory.derivatives[mot][0] = 0;
	trajectory.derivatives[mot][1] = ((15*e1)-(4*e2)+e3)/56;
	trajectory.derivatives[mot][2] = (-e1+(4*e2)-e3)/14;
	trajectory.derivatives[mot][3] = (e1-(4*e2)+(15*e3))/56;
	trajectory.derivatives[mot][4] = 0;
    }
}

/****************************************************************************/

void CLMBase::polCoefficients() {

    for (int mot=0; mot<base->GetMOT()->cnt; mot++) { 
        // currently fixed to 5 motors
	for (int pol=0; pol< trajectory.number_of_points - 1; pol++) {
	    trajectory.coefficients[mot][pol][0] = 
		trajectory.motors[mot][1 + pol] 
		- trajectory.motors[mot][0 + pol];
	    trajectory.coefficients[mot][pol][1] = 
		trajectory.coefficients[mot][pol][0] 
		- (trajectory.dt * trajectory.derivatives[mot][0 + pol]);
	    trajectory.coefficients[mot][pol][2] = 
		(trajectory.dt * trajectory.derivatives[mot][1 + pol]) 
		- trajectory.coefficients[mot][pol][0];
	    trajectory.coefficients[mot][pol][3] = 
		trajectory.coefficients[mot][pol][2] 
		- trajectory.coefficients[mot][pol][1];
	}
    }
}

/****************************************************************************/

TRetLMCalcP CLMBase::calcParameters(double* arr_actpos, 
				    double* arr_tarpos, 
				    double vmax) {

    int shift3 = 32768;
    int shift2 = 1024;
    int shift1 = 64;
    int shift0 = 0;

    trajectory.arr_actpos = arr_actpos;
    trajectory.arr_tarpos = arr_tarpos;

    switch(fillPoints(vmax)) {
	case LMFill_KINEMATICS_ERROR:	
	    return LMCalcP_KINEMATICS_ERROR; 
	    break;
	case LMFill_OUT_OF_RANGE:		
	    return LMCalcP_OUT_OF_RANGE; 
	    break;
    }

    polDeviratives();	
    polCoefficients();

    for (int mot=0; mot<base->GetMOT()->cnt; mot++) {
	for (int pol=0; pol< trajectory.number_of_points - 1; pol++) {

	    // important: in K4D this is rounded instead of casted
	    trajectory.parameters[mot][pol][3] = 
		(short)(pow(0.01,3) 
			* trajectory.coefficients[mot][pol][3] 
			/ pow(trajectory.dt,3) * shift3);
	    trajectory.parameters[mot][pol][2] = 
		(short)((pow(0.01,2) 
			 * (trajectory.coefficients[mot][pol][1] 
			    - trajectory.coefficients[mot][pol][3]) 
			 / pow(trajectory.dt,2)) * shift2); 
	    trajectory.parameters[mot][pol][1] = 
		(short)(((0.01 
			  * (trajectory.coefficients[mot][pol][0] 
			     - trajectory.coefficients[mot][pol][1]))
			 /trajectory.dt) * shift1);
	    trajectory.parameters[mot][pol][0] = 
		(short)(trajectory.motors[mot][pol]);
	}
    } return LMCalcP_SUCCESSFUL;
}

/****************************************************************************/
/****************************************************************************/

void CLMBase::initLM() {

    trajectory.number_of_points = 5;

    short number_of_coefficients = 4;
    short number_of_parameters = 4;

    trajectory.points = new TLM_points[trajectory.number_of_points];

    trajectory.motors = new short* [base->GetMOT()->cnt];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
	trajectory.motors[i] = new short[trajectory.number_of_points];

    trajectory.derivatives = new double* [base->GetMOT()->cnt];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
	trajectory.derivatives[i] = new double[trajectory.number_of_points];

    trajectory.coefficients = new double** [base->GetMOT()->cnt];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
	trajectory.coefficients[i] = 
	    new double* [trajectory.number_of_points-1];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
	for (int j=0; j<trajectory.number_of_points-1; j++)
	    trajectory.coefficients[i][j] = 
		new double[number_of_coefficients];

    trajectory.parameters = new short** [base->GetMOT()->cnt];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
	trajectory.parameters[i] = 
	    new short* [trajectory.number_of_points-1];
    for (int i=0; i<base->GetMOT()->cnt; i++)  
		for (int j=0; j<trajectory.number_of_points-1; j++)
		 trajectory.parameters[i][j] = 
			new short[number_of_parameters];

}

/****************************************************************************/

TRetLM CLMBase::movLM2P(double X1, double Y1, double Z1, 
		      double Al1, double Be1, double Ga1, 
					   double X2, double Y2, double Z2, 
		      double Al2, double Be2, double Ga2, 
		      bool exactflag, double vmax, bool wait, long timeout) {

	// check if the robot buffer is ready to receive a new linear movement

		bool motors_ready = false;

		while (!motors_ready) {

			motors_ready = true;

			for (int idx = 0; idx < base->GetMOT()->cnt - 1; idx++) {

				if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) 
					return LM_MOVING_ERROR;	
  
				motors_ready &= (base->GetMOT()->arr[idx].GetPVP()->msf != 152);
 
				//printf("\nMotor %d motors_ready: %d", idx, motors_ready);
			};
		};

    double		arr_tarpos[6] = {X2, Y2, Z2, Al2, Be2, Ga2};		
    // target position in cartesian units
	double		arr_actpos[6] = {X1, Y1, Z1, Al1, Be1, Ga1};
    // source position in cartesian units

    switch(calcParameters(arr_actpos, arr_tarpos, vmax)) {
	case LMCalcP_KINEMATICS_ERROR: 
	    return LM_KINEMATICS_ERROR; 
	    break;
	case LMCalcP_OUT_OF_RANGE: 
	    return LM_OUT_OF_RANGE; 
    }

    //---------------------------------------------------------------------//
    // Send command L

    byte p[38];	//packet

    for (int mot=0; mot <= base->GetMOT()->cnt-1; mot++) { 
    // it is implemented only for 5 motors

	p[0] = 'L';
	p[1] = mot + 1;
	p[2] = 
	    (byte)(trajectory.motors[mot][trajectory.number_of_points - 1] 
		   >> 8);
	p[3] = 
	    (byte)(trajectory.motors[mot][trajectory.number_of_points - 1]);
	p[4] = (byte)((int)((double)(trajectory.dt)/0.01) >> 8);
	p[5] = (byte)((int)((double)(trajectory.dt)/0.01));

	for (int pol=0; pol< trajectory.number_of_points - 1; pol++) {
	    p[6 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][0] 
				    >> 8); 
	    p[7 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][0]);
	    p[8 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][1] 
				    >> 8);
	    p[9 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][1]);
	    p[10 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][2] 
				     >> 8);
	    p[11 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][2]);
	    p[12 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][3] 
				     >> 8);
	    p[13 + 8 * pol] = (byte)(trajectory.parameters[mot][pol][3]);
	}

	if (base->sendSLMP(p) != SET_NO_ERR) 
	    return LM_COMMUNICATION_ERROR;
    }

    if (base->sendSLM(exactflag) != SET_NO_ERR) 
	return LM_COMMUNICATION_ERROR;

	if (!wait) return LM_TARGET_SET;

	bool pos_reached = true;
	bool motor_blocked = false;

	clock_t t = gettics();
	while (true) {

		if ((1000*(gettics()-t) > timeout * CLOCKS_PER_SEC) && (timeout > 0)){
			return LM_MOVING_ERROR;
		}

		for (int idx=0; idx<5; idx++) {

			if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) {
				return LM_MOVING_ERROR;
			}

			pos_reached &= ABSOLUTE(trajectory.motors[idx][trajectory.number_of_points - 1] - base->GetMOT()->arr[idx].GetPVP()->pos)
				 < base->GetMOT()->arr[idx].GetENL()->enc_tolerance;

			motor_blocked |= (base->GetMOT()->arr[idx].GetPVP()->msf == 40);

		}
		if (motor_blocked) {
			return LM_CRASHED;
		};

		if (pos_reached) {
			return LM_POSITION_REACHED;
		};
	} 

    return LM_TARGET_SET;
}

TRetLM CLMBase::movLM(double X, double Y, double Z, 
		      double Al, double Be, double Ga, 
		      bool exactflag, double vmax, bool wait, long timeout) {

    double		arr_tarpos[6] = {X, Y, Z, Al, Be, Ga};		
    // target position in cartesian units
    double		arr_actpos[6];	
    // current position in cartesian units
    DKApos(arr_actpos);

	return movLM2P(arr_actpos[0], arr_actpos[1], arr_actpos[2], arr_actpos[3], arr_actpos[4], arr_actpos[5],
					arr_tarpos[0], arr_tarpos[1], arr_tarpos[2], arr_tarpos[3], arr_tarpos[4], arr_tarpos[5], 
						exactflag, vmax, wait, timeout);

}

/****************************************************************************/
