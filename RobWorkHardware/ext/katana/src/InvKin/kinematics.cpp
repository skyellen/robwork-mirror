/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Copyright (C) 2004 Thomas Kopfstedt IPT-Robotk/DMS-ES
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


#include "KNI_InvKin/kinematics.h"

#include <math.h>
#include <stdio.h>

#define M_PI	3.14159265358979323846
#define sign(x)	((x<0)? (-1):(1))


void KATANA_VWL (double *laenge, double *position, double *winkel_u) 
{
/* From the angels of the joints, this function calculates the position 
 * and orientation of the end effectors relative to the origin of the coordinate system, 
 * which was put into the midpoint of joint 2, because a shift to the floor or other hights 
 * would only mean a translation. The local and global coordinate systems used as well as the 
 * angle relations and orientations follow the rules of the "Denavit-Hartenberg".
 */

	// declaration of variables
	double	winkel[6];
	int		i;

	// make the needed sign corrections to relate the IK results with 
	// the motors' 2 and 3 sense

	winkel_u[2] *= -1;
	winkel_u[3] *= -1;	

	// Angle translation from degree to degree/Pi
	for (i=0; i<6; i++)
	{
		winkel[i] = winkel_u[i]/180 * M_PI;
	}

	//Calculation of position of the end effectors from the constants and angles
	position[0] = -sin(winkel[0])*(cos(winkel[1])*laenge[1]
			+cos(winkel[1]+winkel[2])*laenge[2] 
			+cos(winkel[1]+winkel[2]+winkel[3])*laenge[3]);
	position[1] =  cos(winkel[0])*(cos(winkel[1])*laenge[1]  
			+cos(winkel[1]+winkel[2])*laenge[2]  
			+cos(winkel[1]+winkel[2]+winkel[3])*laenge[3]);
	position[2] = -sin(winkel[1])*laenge[1] 
			-sin(winkel[1]+winkel[2])*laenge[2] 
			-sin(winkel[1]+winkel[2]+winkel[3])*laenge[3];

	// Calculation of the angles of the end effectors from the constants and angles
	position[3] = winkel[1] + winkel[2] + winkel[3];
	position[4] = winkel[0];
	position[5] = winkel[4];

	// Transformation of the angles from degree to degree/Pi
	position[3] = position[3] * 180 / M_PI;
	position[4] = position[4] * 180 / M_PI;
	position[5] = position[5] * 180 / M_PI;



}


void KATANA_RWL (double *laenge, double *position_u, double *W1, double *W2,
	double *W3, double *W4) 
{
/* This function serves to define the angles of the different joint directions based on 
 * the x,y,z positions of the end effectors and its angles alpha, beta and gamma.
 * To make these calculations possible, the lengths of the segments are used as  
 * additional input informations. The possible angle directions are returned  
* as 4 vectors of the length 6.
 */

	// declaration of variables
	double	position[6];
	int	i, j;
	double	xx, yy, zz, fd;
	double	W[6][4];
	double	inkreisradius, hilf, hilf2, theta_hilf_1, theta_hilf_2,
		theta_hilf_3;

	for (i=0; i<6; i++)
	{
		position[i] = position_u[i];
	}

	// Translation of angles alpha and gamma from degree to degree/Pi
	position[3] = fmod(position[3],360) / 180 * M_PI;
	position[5] = fmod(position[5],360) / 180 * M_PI;


	// Calculation of  W6  (W means "Winkel", in English "Angle")

	// As the joint directions of joint 6 are not defined by the inverse kinematics,
	// the angle 0 is returned for this angle. If a value different from 0 is returned, it 
	//may be the number of an error, because this function also serves for errors printing

	W[5][0] = 0;
	W[5][1] = 0;
	W[5][2] = 0;
	W[5][3] = 0;

/* Calculation of joint position of joint 1:
* The angle of this joint results from the x coordinate and y coordinates, because joint 1 is 
* the only joint which can rotate the arm around the z axis. Joint 5 is able to do so only in 
* special cases, but has no influence on the coordinates of the manipulator, because joint 5 is 
* located at the end effector. There are, theoretically, two angles  for each position, W11 and an 
* angle turned by 180 degree. Additionally, there is a special case, when the target point is lying 
* exactly above the origin of the coordinate system.
 */
	if (((int)(position[0]) == 0) & ((int)(position[1]) == 0)) 
	{
		// Target coordinate is located directly above
		// the origine of the coordinate system, angle is not required, therefore
		// W11 = W12 is returned.
		W[0][0] = 0;
		W[0][1] = 0;
		W[0][2] = 0;
		W[0][3] = 0;
	}
	else 
	{
		W[0][0] = atan2(position[1],position[0]) - M_PI / 2.0;
		W[0][1] = W[0][0];
		W[0][2] = W[0][0] - M_PI;
		W[0][3]	= W[0][2];         
	}

/* Calculation of W5
 * The position of joint 5 is equal to the turning angle Gamma of the end effector
 * and this can be calculated independantly of the other angles. There is only one 
 * special case, when joint 5 is in the line of joint 1. This is the case when the whole 
 * manipulator arm is extended only in z direction (that is, when the target coordinate 
 * has only a z component). In this case, joint 5 is turned according to Gamma. 
 * If the rotating limitation of joint 5 does not allow to reach the position, joint 1 may help.     
 * This will be the task of the function which checks with regards to collosions and not allowed   
 * angles.
 */
	W[4][0] = position[5];
	W[4][1] = position[5];
	W[4][2] = position[5];
	W[4][3] = position[5];

/* Calculation of W2, W3, W4 (general variables)
 * Calculation of the distances between joints 2 and 4. This is done because
 * joints 2, 3 and 4 move all along the same coordinate axis, and because 
 * after joint 4 already the end effector follows. As the end effector has to be 
 * directed to the target point according to the angle alpha
 * the distance from joint 2 to joint 4 is sufficient to calculated
 * the angle positions of these joints.
 */
	xx = position[0] + sin(W[0][0])*cos(position[3])*laenge[3];   
	yy = position[1] - cos(W[0][0])*cos(position[3])*laenge[3];
	zz = position[2] + sin(position[3])*laenge[3];         
	fd = sqrt(xx*xx + yy*yy); 

//The following equations origin from the book "Introduction to robotics" and 
//were adjusted and reduced to this manipulator arm with the help of mathematic books.

	// half circumference of the triangle: joint 2, joint 3, joint 4
	hilf = 1.0/2.0*(laenge[1]+laenge[2]+sqrt(fd*fd+zz*zz));

	// inner circle radius of joint 2 to joint 4
	hilf2 = ((hilf-laenge[1])*(hilf-laenge[2])*(hilf-sqrt(fd*fd+zz*zz)))
		/ hilf;
	if (hilf2 > 0) 
	{	
		inkreisradius = sqrt(hilf2);
	}
	else 
	{
		inkreisradius = sqrt(-hilf2);
	}

	// angle position of joint 2 (segment 2) in relation to the angle position of the 
	// inner circle radius
	theta_hilf_1 = 2.0*atan2(inkreisradius,(hilf-laenge[1]));

	// angle position of joint 3 (segment 3) in relation to the angle position of the 
	// inner circle radius
	theta_hilf_2 = 2.0*atan2(inkreisradius,(hilf-laenge[2]));

	//angle position of the inner circle radius
	theta_hilf_3 = atan2(zz,fd);

/* As for each of the 2 possible solutions for joint 1 there are, theoretically, 
 * 2 solutions available for the joint combinations 2,3,4, there are 4 solutions 
 * available for the joint combinations 2,3,4. They will be considered in the 
 * following. The angle position of joint 4 results always from the two joint 
 * angles of the joints 2 and 3 and the between-angle alpha. 
*/

	// Calculation of W2, W3, W4 (upper variant) at W11
	W[1][0] = -(theta_hilf_3 + theta_hilf_2);
	W[2][0] = (-(theta_hilf_3 - theta_hilf_1)) - W[1][0];
	W[3][0] = fmod((position[3] - W[1][0] - W[2][0]),(2.0 * M_PI));

	// Calculation of W2, W3, W4 (under variant) at W11
	W[1][1] = -(theta_hilf_3 - theta_hilf_2);
	W[2][1] = (-(theta_hilf_3 + theta_hilf_1)) - W[1][1];
	W[3][1] = fmod((position[3] - W[1][1] - W[2][1]),(2.0 * M_PI));

	// Calculation of W2, W3, W4 (upper variant) at W12
	W[1][2] = -(M_PI + (-(theta_hilf_3 + theta_hilf_2)));
	W[2][2] = W[2][1];
	W[3][2] = fmod((M_PI + position[3] - W[1][2] - W[2][1]),(2.0 * M_PI));

	// Calculation of W2, W3, W4 (under variant) at W12
	W[1][3] = -(M_PI + (-(theta_hilf_3 - theta_hilf_2)));
	W[2][3] = W[2][0];
	W[3][3] = fmod((M_PI + position[3] - W[1][3] - W[2][0]),(2.0 * M_PI));

	// Translation of the angles from degree/Pi to degree
	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			W[i][j] = W[i][j] * 180 / M_PI;
		}
	}

	// Adjustment of the angle matrix, so that all angles result between  +/- 180 degree
	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			if (W[i][j] > 180) 
			{
				W[i][j] = W[i][j] - 360.0;
			}
			if (W[i][j] < -180) 
			{
				W[i][j] = W[i][j] + 360.0;
			}
		}
	}


	for (i=0; i<6; i++)
	{
		W1[i] = W[i][0];
		W2[i] = W[i][1];
		W3[i] = W[i][2];
		W4[i] = W[i][3];
	}


	if ((sign((int)(position[0])) != sign((int)(xx))) 
		|| (sign((int)(position[1])) != sign((int)(yy))))
	{
		W1[1] = W[1][2];
		W2[1] = W[1][3];
		W3[1] = W[1][0];
		W4[1] = W[1][1];
		W1[2] = W[2][2];
		W2[2] = W[2][3];
		W3[2] = W[2][0];
		W4[2] = W[2][1];
		W1[3] = W[3][2] + 180.0;
		W2[3] = W[3][3] + 180.0;
		W3[3] = W[3][0] + 180.0;
		W4[3] = W[3][1] + 180.0;
	}	

	// make the needed sign corrections to relate the IK results with 
	// the motors' 2 and 3 sense

	W1[2] *= -1; 
	W1[3] *= -1;
	W2[2] *= -1;
	W2[3] *= -1;
	W3[2] *= -1;
	W3[3] *= -1;
	W4[2] *= -1;
	W4[3] *= -1;

}


void KATANA_RWL_V2 (double *laenge, double *position_u, double *W1, double *W2,
	double *W3, double *W4) 
{


	double	position[6];
	int	i, j;
	double	xx, yy, zz, fd;
	double	W[6][4];
	double	inkreisradius, hilf, hilf2, theta_hilf_1, theta_hilf_2,
		theta_hilf_3;

	for (i=0; i<6; i++)
	{
		position[i] = position_u[i];
	}

	position[3] = fmod(position[3],360) / 180 * M_PI;
	position[4] = fmod(position[4],360) / 180 * M_PI;
	position[5] = fmod(position[5],360) / 180 * M_PI;

	W[5][0] = 0;
	W[5][1] = 0;
	W[5][2] = 0;
	W[5][3] = 0;

	W[0][0] = position[4];
	W[0][1] = position[4];
	W[0][2] = position[4] + M_PI;
	W[0][3] = position[4] + M_PI;

	W[4][0] = position[5];
	W[4][1] = position[5];
	W[4][2] = position[5];
	W[4][3] = position[5];

	xx = position[0] + sin(W[0][0])*cos(position[3])*laenge[3];   
	yy = position[1] - cos(W[0][0])*cos(position[3])*laenge[3];
	zz = position[2] + sin(position[3])*laenge[3];         
	fd = sqrt(xx*xx + yy*yy); 

	hilf = 1.0/2.0*(laenge[1]+laenge[2]+sqrt(fd*fd+zz*zz));

	hilf2 = ((hilf-laenge[1])*(hilf-laenge[2])*(hilf-sqrt(fd*fd+zz*zz)))
		/ hilf;
	if (hilf2 > 0) 
	{	
		inkreisradius = sqrt(hilf2);
	}
	else 
	{
		inkreisradius = sqrt(-hilf2);
	}

	theta_hilf_1 = 2.0*atan2(inkreisradius,(hilf-laenge[1]));

	theta_hilf_2 = 2.0*atan2(inkreisradius,(hilf-laenge[2]));

	theta_hilf_3 = atan2(zz,fd);

	W[1][0] = -(theta_hilf_3 + theta_hilf_2);
	W[2][0] = (-(theta_hilf_3 - theta_hilf_1)) - W[1][0];
	W[3][0] = fmod((position[3] - W[1][0] - W[2][0]),(2.0 * M_PI));

	W[1][1] = -(theta_hilf_3 - theta_hilf_2);
	W[2][1] = (-(theta_hilf_3 + theta_hilf_1)) - W[1][1];
	W[3][1] = fmod((position[3] - W[1][1] - W[2][1]),(2.0 * M_PI));

	W[1][2] = -(M_PI + (-(theta_hilf_3 + theta_hilf_2)));
	W[2][2] = W[2][1];
	W[3][2] = fmod((M_PI + position[3] - W[1][2] - W[2][1]),(2.0 * M_PI));

	W[1][3] = -(M_PI + (-(theta_hilf_3 - theta_hilf_2)));
	W[2][3] = W[2][0];
	W[3][3] = fmod((M_PI + position[3] - W[1][3] - W[2][0]),(2.0 * M_PI));

	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			W[i][j] = W[i][j] * 180 / M_PI;
		}
	}

	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			if (W[i][j] > 180) 
			{
				W[i][j] = W[i][j] - 360.0;
			}
			if (W[i][j] < -180) 
			{
				W[i][j] = W[i][j] + 360.0;
			}
		}
	}

	for (i=0; i<6; i++)
	{
		W1[i] = W[i][0];
		W2[i] = W[i][1];
		W3[i] = W[i][2];
		W4[i] = W[i][3];
	}

	if ((sign((int)(position[0])) != sign((int)(xx))) 
		|| (sign((int)(position[1])) != sign((int)(yy))))
	{
		W1[1] = W[1][2];
		W2[1] = W[1][3];
		W3[1] = W[1][0];
		W4[1] = W[1][1];
		W1[2] = W[2][2];
		W2[2] = W[2][3];
		W3[2] = W[2][0];
		W4[2] = W[2][1];
		W1[3] = W[3][2] + 180.0;
		W2[3] = W[3][3] + 180.0;
		W3[3] = W[3][0] + 180.0;
		W4[3] = W[3][1] + 180.0;
	}	

	// make the needed sign corrections to relate the IK results with 
	// the motors' 2 and 3 sense

	W1[2] *= -1; 
	W1[3] *= -1;
	W2[2] *= -1;
	W2[3] *= -1;
	W3[2] *= -1;
	W3[3] *= -1;
	W4[2] *= -1;
	W4[3] *= -1;
}

