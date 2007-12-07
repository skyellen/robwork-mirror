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
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
/******************************************************************************************************************/
#include "KNI/dllexport.h"
/******************************************************************************************************************/

DLLDIR_IK void KATANA_VWL    (double *laenge, double *position,   double *winkel_u);
DLLDIR_IK void KATANA_RWL    (double *laenge, double *position_u, double *W1, double *W2, double *W3, double *W4);
DLLDIR_IK void KATANA_RWL_V2 (double *laenge, double *position_u, double *W1, double *W2, double *W3, double *W4);

/******************************************************************************************************************/
#endif /*_KINEMATICS_H*/
/******************************************************************************************************************/
