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
#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_
/******************************************************************************************************************/
#include "dllexport.h"
#ifdef WIN32
	#include <time.h>
#else
	#include <sys/times.h>
	#define CLOCKS_PER_SEC 100
#endif
/******************************************************************************************************************/

template<typename _T> inline _T ABSOLUTE(_T p) { return (p<0) ? -p : p; }

inline void SLEEP(long diff) {
#ifdef WIN32
	clock_t t = clock();
	while (1000 * (clock() - t) < diff * CLOCKS_PER_SEC) {}
#else
	tms time;
	clock_t t = times(&time);
	while (1000 * ((times(&time) - t)) < diff * CLOCKS_PER_SEC) {}
#endif
}

inline clock_t gettics() {
#ifdef WIN32
	return clock();
#else
	tms time;
	return times(&time);
#endif
}


/******************************************************************************************************************/
#endif //_KMLEXT_H_
/******************************************************************************************************************/
