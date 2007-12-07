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
#include "KNI_InvKin/ikBase.h"
#include "KNI_InvKin/kinematics.h"
#include "KNI/functions.h"

// macro to compare variables double 
#define compare(x,y)	((x - y < 0.1 && x - y >= 0) || (y - x < 0.1 && y - x >= 0)) 		

/****************************************************************************/

void CikBase::DKApos(double* position) {

	//double arr_pos[6];		//position array for kinematics
	double arr_del[6];

	for (int i=0; i<base->GetMOT()->cnt; i++) {
		base->GetMOT()->arr[i].recvPVP();			
		arr_del[i] = encodertodegree(i,base->GetMOT()->arr[i].GetPVP()->pos);
	};			

	KATANA_VWL(base->GetEFF()->arr_segment,position,arr_del);	

}

TRetIKSol CikBase::IKSolution(short ns, double** arr_angle) {

	if (ns <= 0 || ns > 4 || ns > ikr.ps) return IKSol_SOLUTION_NOT_AVAILABLE;

	short counter = 0;

	short i=-1;
	while (counter != ns) {
		if (ikr.valid[ikr.ranking[++i]]) 
			counter++;
	};

	*arr_angle = ikr.arr_angle[ikr.ranking[i]];

	return IKSol_SUCCESSFUL;

}

TRetIKCal CikBase::IKCalculate(double X, double Y, double Z, double Al, double Be, double Ga) {

	// Calculates IK, and fills ikr

	ikr.ps = 0; // ps: number of possible solution that are reachable by the robot

	/*------------------------------------------------------------------------------------------*/
	//	use KATANA_RWL to get all four possible angle combinations to reach the given position

	TKatEFF* eff = base->GetEFF();
	const TKatMOT* mot = base->GetMOT();

	double arr_position[] = {X,Y,Z,Al,Be,Ga};
	KATANA_RWL(eff->arr_segment,arr_position,
		ikr.arr_angle[0],
		ikr.arr_angle[1],
		ikr.arr_angle[2],
		ikr.arr_angle[3]);

	/*------------------------------------------------------------------------------------------*/
	// calculate if the solutions match with the input, if not
	// the entrance is out of range	
	// check out if the solutions are inside the motors' range

	bool solution_match = false;
	double arr_del[6];
	double arr_pos[6];

	ikr.ps = 0;
	
	for (int i=0; i<4; i++) {

		arr_del[0] = ikr.arr_angle[i][0];
		arr_del[1] = ikr.arr_angle[i][1];
		arr_del[2] = ikr.arr_angle[i][2];
		arr_del[3] = ikr.arr_angle[i][3];
		arr_del[4] = ikr.arr_angle[i][4];
		arr_del[5] = ikr.arr_angle[i][5];

		KATANA_VWL(eff->arr_segment,arr_pos,arr_del);

		ikr.valid[i] = (compare(arr_pos[0],X)) && (compare(arr_pos[1],Y)) && (compare(arr_pos[2],Z));

		for (int j=0; j<mot->cnt; j++) {

			ikr.valid[i] &= checkENLD(j, (double)ikr.arr_angle[i][j]);
		};

		if (ikr.valid[i]) ikr.ps++;

		solution_match |= ikr.valid[i];
	};

	/*------------------------------------------------------------------------------------------*/
	// fills act_pos[] with the current position in degree units

	double* act_pos = new double[mot->cnt];	
	for (int idx=0; idx<mot->cnt; idx++) {

		if (mot->arr[idx].recvPVP() != RECV_NO_ERR) {
			return IKCal_RECVRMP_FAILED;
		} SLEEP(25);
		
		act_pos[idx] = (double)((double)mot->arr[idx].GetPVP()->pos * (double)map[idx].enc_to_deg);
	}

	double arr_angledif[4];			
	for(int j=0; j<4; j++) {
		arr_angledif[j] = 0.0;
	}

	/*------------------------------------------------------------------------------------------*/
	// calculate the differences between the current and target position of each 
	// possible solution

	for(int j=0; j<4; j++) {		
			for (int idx=0; idx<mot->cnt; idx++) {
				arr_angledif[j] += ABSOLUTE(act_pos[idx] - ikr.arr_angle[j][idx]); 
			};

	}

	/*------------------------------------------------------------------------------------------*/
	// sort the solutions using an array of indexes

	short temp;
	for (int i=0; i<4; i++) ikr.ranking[i] = i;

	for (int i=3; i >= 0; i--) {
		for (int j=1; j<=i; j++) {
			if ( arr_angledif[ ikr.ranking[j - 1] ] > arr_angledif[ ikr.ranking[j]] ) {
				temp = ikr.ranking[ j ];
				ikr.ranking[ j ] = ikr.ranking[ j - 1 ];
				ikr.ranking[ j - 1 ] = temp;
			};

		}
	}
		
	if (!solution_match) return IKCal_OUT_OF_RANGE;

	return IKCal_SUCCESSFUL;
}


TRetIK CikBase::IKMove(short ns, bool wait, long timeout) {

	double* arr_angle = new double[6];

	if (IKSolution(ns, &arr_angle) == IKSol_SOLUTION_NOT_AVAILABLE) return IK_OUT_OF_RANGE;

	movDegrees(0, arr_angle[0]);
	movDegrees(1, arr_angle[1]);
	movDegrees(2, arr_angle[2]);
	movDegrees(3, arr_angle[3]);
	movDegrees(4, arr_angle[4]);
	//movDegrees(5, arr_angle[5]);	

	/*-----------------------------------------------------------------------------------------*/
	// If wait is true, check if the target position is reached

	if (!wait) return IK_TARGET_SET;

	bool pos_reached;

	clock_t t = gettics();
	while (true) {

		if ((1000*(gettics()-t) > timeout * CLOCKS_PER_SEC) && (timeout > 0)){
			return IK_MOVING_ERROR;
		}
		//-------------------------------------------------------------//
		SLEEP(2);	//give enough time for the katana to respond
		//-------------------------------------------------------------//

		pos_reached = true;
		for (int idx=0; idx<5; idx++) {

			if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) {
				return IK_MOVING_ERROR;
			}

			pos_reached &= ABSOLUTE(base->GetMOT()->arr[idx].GetTPS()->tarpos - base->GetMOT()->arr[idx].GetPVP()->pos)
				 < base->GetMOT()->arr[idx].GetENL()->enc_tolerance;
		}

		if (pos_reached) {
			return IK_POSITION_REACHED;
		};

		//-------------------------------------------------------------//
	}

	return IK_TARGET_SET;

}

TRetIK CikBase::IKGoto(double X, double Y, double Z, double Al, double Be, double Ga,  bool wait, long timeout) {

	/*------------------------------------------------------------------------------------------*/
	//	use KATANA_RWL to get all four possible angle combinations to reach the given position

	TKatEFF* eff = base->GetEFF();
	double arr_position[] = {X,Y,Z,Al,Be,Ga};
	KATANA_RWL(eff->arr_segment,arr_position,
		ikr.arr_angle[0],
		ikr.arr_angle[1],
		ikr.arr_angle[2],
		ikr.arr_angle[3]);

	/*------------------------------------------------------------------------------------------*/
	// calculate if the solutions match with the input, if not
	// the entrance is out of range

	bool solution_match = false;
	double arr_del[6];
	double arr_pos[6];
	
			for (int i=0; i<4; i++) {

				arr_del[0] = ikr.arr_angle[i][0];
				arr_del[1] = ikr.arr_angle[i][1];
				arr_del[2] = ikr.arr_angle[i][2];
				arr_del[3] = ikr.arr_angle[i][3];
				arr_del[4] = ikr.arr_angle[i][4];
				arr_del[5] = ikr.arr_angle[i][5];

				KATANA_VWL(eff->arr_segment,arr_pos,arr_del);

				solution_match |= (compare(arr_pos[0],X)) && (compare(arr_pos[1],Y)) && (compare(arr_pos[2],Z));
			};

			if (!solution_match) return IK_OUT_OF_RANGE;

	/*------------------------------------------------------------------------------------------*/

	const TKatMOT* mot = base->GetMOT();

	/*------------------------------------------------------------------------------------------*/
	// fills act_pos[] with the current position in degree units

	double* act_pos = new double[mot->cnt];	
	for (int idx=0; idx<mot->cnt; idx++) {

		if (mot->arr[idx].recvPVP() != RECV_NO_ERR) {
			return IK_RECVRMP_FAILED;
		} SLEEP(25);
		
		act_pos[idx] = (double)((double)mot->arr[idx].GetPVP()->pos * (double)map[idx].enc_to_deg);
	}

	double arr_angledif[4];			
	for(int j=0; j<4; j++) {
		arr_angledif[j] = 0.0;
	}

	/*------------------------------------------------------------------------------------------*/
	// calculate the differences between the current and target position of each 
	// possible solution

	for(int j=0; j<4; j++) {		
			for (int idx=0; idx<mot->cnt; idx++) {
				arr_angledif[j] += ABSOLUTE(act_pos[idx] - ikr.arr_angle[j][idx]); 
			};

	}

	/*------------------------------------------------------------------------------------------*/
	// choose the closest and reachable (not out of range) solution

	int option;	
	double floormin = 0;
	int solutions_cheked = 0;
	bool range_check = false;

	while(solutions_cheked < 4 && range_check == false) {

		solutions_cheked++;

		double min = 180 * mot->cnt;
		for(int i = 0; i < 4; i++)
			if(arr_angledif[i] < min && arr_angledif[i] > floormin) {
				min = arr_angledif[i];
				option = i;
			}

		range_check = true;
		for (int i=0; i<base->GetMOT()->cnt; i++) {
			range_check &= checkENLD(i, (double)ikr.arr_angle[option][i]);
		}

		if (range_check == false) floormin = min;
	}

	/*------------------------------------------------------------------------------------------*/
	// move to the desired position

	if (!range_check) return IK_OUT_OF_RANGE;

	movDegrees(0, ikr.arr_angle[option][0]);
	movDegrees(1, ikr.arr_angle[option][1]);
	movDegrees(2, ikr.arr_angle[option][2]);
	movDegrees(3, ikr.arr_angle[option][3]);
	movDegrees(4, ikr.arr_angle[option][4]);
	//movDegrees(5, ikr.arr_angle[option][5]);	

	/*-----------------------------------------------------------------------------------------*/
	// If wait is true, check if the target position is reached

	if (!wait) return IK_TARGET_SET;

	bool pos_reached;
	bool motor_blocked;

	clock_t t = gettics();
	while (true) {

		if ((1000*(gettics()-t) > timeout * CLOCKS_PER_SEC) && (timeout > 0)){
			return IK_MOVING_ERROR;
		}
		//-------------------------------------------------------------//
		SLEEP(2);	//give enough time for the katana to respond
		//-------------------------------------------------------------//

		pos_reached = true;
		motor_blocked = false;	

		for (int idx=0; idx<5; idx++) {

			if (base->GetMOT()->arr[idx].recvPVP() != RECV_NO_ERR) {
				return IK_MOVING_ERROR;
			}

			pos_reached &= ABSOLUTE(base->GetMOT()->arr[idx].GetTPS()->tarpos - base->GetMOT()->arr[idx].GetPVP()->pos)
				 < base->GetMOT()->arr[idx].GetENL()->enc_tolerance;

			motor_blocked |= (base->GetMOT()->arr[idx].GetPVP()->msf == 40);

		}

		if (motor_blocked) {
			return IK_CRASHED;
		};

		if (pos_reached) {
			return IK_POSITION_REACHED;
		};

		//-------------------------------------------------------------//
	}

	return IK_TARGET_SET;

}

