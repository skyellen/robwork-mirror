/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Copyright (C) 2004 Thomas Kopfstedt IPT-Robotik / DMS-ES
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


#ifndef _KOPFSTEDT_KATANA_BERECHNUNG_H
  #include "KNIHeaders/katana_berechnung.h"
#endif

#include <math.h>
#include <stdio.h>

#define M_PI	3.14159265358979323846
#define sign(x)	((x<0)? (-1):(1))

void KATANA_VWL (double *laenge, double *position, double *winkel_u) 
{
/* Diese Funktion berechnet aus den Winkelstellungen der Gelenke die Position
 * und Orientierung des Endeffektors relativ zum Koordinatenursprung, welcher
 * in die Mitte des Gelenkes 2 gelegt wurde, da der einzige Unterschied zum 
 * Koordinatenursprung am Boden usw. nur eine Verschiebung bedeuten würde. Die
 * verwendenten lokalen und globalen Koordinatensysteme sowie die 
 * Winkelbeziehungen und Orientierungen entsprechen den Regeln von 
 * "Denavit-Hartenberg".
 */

	// Variabelndeklaration
	double	winkel[6];
	int		i;

	// Umwandlung der Winkel von Grad in Grad/Pi
	for (i=0; i<6; i++)
	{
		winkel[i] = winkel_u[i]/180 * M_PI;
	}

	// Berechnung der Position des Endeffektors aus den Konstanten und 
	// Winkeln
	position[0] = -sin(winkel[0])*(cos(winkel[1])*laenge[1]
			+cos(winkel[1]+winkel[2])*laenge[2] 
			+cos(winkel[1]+winkel[2]+winkel[3])*laenge[3]);
	position[1] =  cos(winkel[0])*(cos(winkel[1])*laenge[1]  
			+cos(winkel[1]+winkel[2])*laenge[2]  
			+cos(winkel[1]+winkel[2]+winkel[3])*laenge[3]);
	position[2] = -sin(winkel[1])*laenge[1] 
			-sin(winkel[1]+winkel[2])*laenge[2] 
			-sin(winkel[1]+winkel[2]+winkel[3])*laenge[3];

	// Berechnung der Winkelstellung des Endeffektors aus den Konstanten
	// und Winkeln
	position[3] = winkel[1] + winkel[2] + winkel[3];
	position[4] = winkel[0];
	position[5] = winkel[4];

	// Umwandlung der Winkel von Grad/Pi in Grad
	position[3] = position[3] * 180 / M_PI;
	position[4] = position[4] * 180 / M_PI;
	position[5] = position[5] * 180 / M_PI;
}


void KATANA_RWL (double *laenge, double *position_u, double *W1, double *W2,
	double *W3, double *W4) 
{
/* Diese Funktion dient zur Bestimmung der Winkelstellung der einzelnen Gelenk-
 * stellungen auf Grundlage der x,y,z Position des Enteffektors und dessen 
 * Winkel alpha, beta und gamma.
 * Um diese Berechnungen durchführen zu können werden zusätzlich zu den 
 * Eingangsinformationen noch die Segmentlängen verwendet. Die möglichen 
 * Winkelstellung werden in 4 Vektoren der Länge 6 zurückgeliefert.
 */

	// Variabelndeklaration
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

	// Umwandlung der Winkel alpha und gamma von Grad in Grad/pi
	position[3] = fmod(position[3],360) / 180 * M_PI;
	position[5] = fmod(position[5],360) / 180 * M_PI;


// Bestimmung von W6
// Da die Gelenkstellung von Gelenk 6 nicht durch die inverse Kinematik
// bestimmt wird, wird für dieses Gelenk der Winkel 0° zurückgeliefert, wenn
// ein Wert ungleich 0° zurückgeliefert wird, dann nur deshalb, weil mit
// dieser Variabeln zusätzlich auch noch Fehler angezeigt werden.
	W[5][0] = 0;
	W[5][1] = 0;
	W[5][2] = 0;
	W[5][3] = 0;

// Bestimmung von W1
/* Bestimmung der Gelenkstellung von Gelenk 1:
 * Der Winkel dieses Gelenkes ergibt sich schlüssig aus der x-Koordinate und
 * y-Koordinate, da Gelenk 1 das einzige Gelenk ist, was den Manipulatorarm
 * um die z-Achse drehen kann, Gelenk 5 kann dies zwar in Spezialfällen
 * auch, hat aber auf die Koordinaten des Manipulators keinen Einfluss, da
 * sich dieses Gelenk am Endeffektor befindet. Es gibt theoretisch für jede
 * Position zwei Winkelstellungen, einmal die W11 und dann eine um 180°
 * gedrehte Winkelstellung. Außerdem gibt es noch den Sonderfall, wenn der
 * Zielpunkt genau oberhalb des Koordinatenursprungs liegt.
 */
	if (((int)(position[0]) == 0) & ((int)(position[1]) == 0)) 
	{
		// Zielkoordinate befindet sich direkt oberhalb des
		// Koordinatenursprungs, Winkelstellung ist egal, deshalb
		// wird W11 = W12 zurückgeliefert.
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

/* Bestimmung von W5
 * Die Stellung von Gelenk 5 entspricht dem Drehwinkel Gamma des
 * Endeffektors und kann somit unabhängig von den anderen Winkeln berechnet
 * werden, es gibt nur einen Spezialfall, indem Gelenk 5 mit Gelenk 1
 * zusammenfällt. Dies ist dann der Fall, wenn sich der gesamte
 * Manipulatorarm nur in z-Richtung erstreckt (wenn also die Zielkoordinate
 * nur einen z-Anteil aufweist. In diesem Fall wird Gelenk 5 entsprechend
 * Gamma gedreht und wenn es aufgrund der Beschränkungen von Gelenk 5 nicht
 * möglich ist die Position zu erreichen wird Gelenk 1 zu Hilfe genommen,
 * bis diese Position erreicht werden kann. Dies ist allerdings dann Aufgabe
 * der Funktion, welche auf Kollision und unzulässige Winkelstellungen	
 * prüft.
 */
	W[4][0] = position[5];
	W[4][1] = position[5];
	W[4][2] = position[5];
	W[4][3] = position[5];

/* Bestimmung von W2, W3, W4 (allgemeine Variabeln)
 * Bestimmung der Abstände zwischen Gelenk 2 und Gelenk 4. Dies wird deshalb
 * gemacht, weil sich die Gelenke 2, 3 und 4 alle entlang der selben
 * Koordinatenachse bewegen und nach Gelenk 4 schon der Endeffektor folgt.
 * Da dieser gemäß dem Winkel alpha zum Zielpunkt ausgerichtet sein muß,
 * reicht die Entfernung von Gelenk 2 zu Gelenk 4 um die Winkelstellungen
 * von diesen Gelenken zu berechnen.
 */
	xx = position[0] + sin(W[0][0])*cos(position[3])*laenge[3];   
	yy = position[1] - cos(W[0][0])*cos(position[3])*laenge[3];
	zz = position[2] + sin(position[3])*laenge[3];         
	fd = sqrt(xx*xx + yy*yy); 

// Nachfolgende Formeln entstammen der Buch "Introduction to robotics" und	
// wurde mittels Mathematischer Formelsammlung auf diesen Manipulatorarm
// abgespeckt und angepasst. 

	// Halber Umfang des Dreiecks: Gelenk 2, Gelenk 3, Gelenk 4
	hilf = 1.0/2.0*(laenge[1]+laenge[2]+sqrt(fd*fd+zz*zz));

	// Inkreisradius vom Gelenk 2 zum Gelenk 4
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

	// Winkelstellung von Gelenk 2 (Segment 2) gegenüber der Winkelstellung
	// des Inkreisradius
	theta_hilf_1 = 2.0*atan2(inkreisradius,(hilf-laenge[1]));

	// Winkelstellung von Gelenk 3 (Segment 3) gegenüber der Winkelstellung
	// des Inkreisradius
	theta_hilf_2 = 2.0*atan2(inkreisradius,(hilf-laenge[2]));

	// Winkelstellung des Inkreisradius
	theta_hilf_3 = atan2(zz,fd);


/* Da es für jede Variante des Gelenkes 1 theoretisch 2 Lösungen für die
 * Gelenkekombination 2,3,4 gibt und es theoretisch stehts zwei mögliche
 * Lösungen für das Gelenk 1 gibt, gibt es für die Gelenkekombination 2,3,4
 * somit insgesamt 4 mögliche Lösungen. Welche jetzt nachfolgend betrachtet 
 * werden. Die Winkelstellung des Gelenkes 4 ergibt sich dabei stehts aus
 * den beiden Gelenkwinkeln der Gelenke 2 und 3 und dem Zielwinkel alpha.
 */

	// Bestimmung von W2, W3, W4 (Obere Variante) bei W11
	W[1][0] = -(theta_hilf_3 + theta_hilf_2);
	W[2][0] = (-(theta_hilf_3 - theta_hilf_1)) - W[1][0];
	W[3][0] = fmod((position[3] - W[1][0] - W[2][0]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Untere Variante) bei W11
	W[1][1] = -(theta_hilf_3 - theta_hilf_2);
	W[2][1] = (-(theta_hilf_3 + theta_hilf_1)) - W[1][1];
	W[3][1] = fmod((position[3] - W[1][1] - W[2][1]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Obere Variante) bei W12
	W[1][2] = -(M_PI + (-(theta_hilf_3 + theta_hilf_2)));
	W[2][2] = W[2][1];
	W[3][2] = fmod((M_PI + position[3] - W[1][2] - W[2][1]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Untere Variante) bei W12
	W[1][3] = -(M_PI + (-(theta_hilf_3 - theta_hilf_2)));
	W[2][3] = W[2][0];
	W[3][3] = fmod((M_PI + position[3] - W[1][3] - W[2][0]),(2.0 * M_PI));

	// Umwandlung der Winkel von Grad/Pi in Grad
	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			W[i][j] = W[i][j] * 180 / M_PI;
		}
	}

	// Anpassung der Winkelmatrix, so dass alle Winkel zwischen +/- 180°
	// liegen
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

	// Übergabe der Winkelstellungen an die Rückgabevektoren
	for (i=0; i<6; i++)
	{
		W1[i] = W[i][0];
		W2[i] = W[i][1];
		W3[i] = W[i][2];
		W4[i] = W[i][3];
	}

	// Wenn zwischen den Gelenken 2 und 4 eine negative Entfernung liegt,
	// so müssen die Winkelmatrizen noch nachträglich darauf angepasst 
	// werden.
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
}


void KATANA_RWL_V2 (double *laenge, double *position_u, double *W1, double *W2,
	double *W3, double *W4) 
{
/* Diese Funktion dient zur Bestimmung der Winkelstellung der einzelnen Gelenk-
 * stellungen auf Grundlage der x,y,z Position des Enteffektors und dessen 
 * Winkel alpha, beta und gamma.
 * Um diese Berechnungen durchführen zu können werden zusätzlich zu den 
 * Eingangsinformationen noch die Segmentlängen verwendet. Die möglichen 
 * Winkelstellung werden in 4 Vektoren der Länge 6 zurückgeliefert.
 */

	// Variabelndeklaration
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

	// Umwandlung der Winkel alpha und gamma von Grad in Grad/pi
	position[3] = fmod(position[3],360) / 180 * M_PI;
	position[4] = fmod(position[4],360) / 180 * M_PI;
	position[5] = fmod(position[5],360) / 180 * M_PI;


// Bestimmung von W6
// Da die Gelenkstellung von Gelenk 6 nicht durch die inverse Kinematik
// bestimmt wird, wird für dieses Gelenk der Winkel 0° zurückgeliefert, wenn
// ein Wert ungleich 0° zurückgeliefert wird, dann nur deshalb, weil mit
// dieser Variabeln zusätzlich auch noch Fehler angezeigt werden.
	W[5][0] = 0;
	W[5][1] = 0;
	W[5][2] = 0;
	W[5][3] = 0;

// Bestimmung von W1
/* Bestimmung der Gelenkstellung von Gelenk 1: */
  
	W[0][0] = position[4];
	W[0][1] = position[4];
	W[0][2] = position[4] + M_PI;
	W[0][3] = position[4] + M_PI;

/* Bestimmung von W5
 * Die Stellung von Gelenk 5 entspricht dem Drehwinkel Gamma des
 * Endeffektors und kann somit unabhängig von den anderen Winkeln berechnet
 * werden, es gibt nur einen Spezialfall, indem Gelenk 5 mit Gelenk 1
 * zusammenfällt. Dies ist dann der Fall, wenn sich der gesamte
 * Manipulatorarm nur in z-Richtung erstreckt (wenn also die Zielkoordinate
 * nur einen z-Anteil aufweist. In diesem Fall wird Gelenk 5 entsprechend
 * Gamma gedreht und wenn es aufgrund der Beschränkungen von Gelenk 5 nicht
 * möglich ist die Position zu erreichen wird Gelenk 1 zu Hilfe genommen,
 * bis diese Position erreicht werden kann. Dies ist allerdings dann Aufgabe
 * der Funktion, welche auf Kollision und unzulässige Winkelstellungen	
 * prüft.
 */
	W[4][0] = position[5];
	W[4][1] = position[5];
	W[4][2] = position[5];
	W[4][3] = position[5];

/* Bestimmung von W2, W3, W4 (allgemeine Variabeln)
 * Bestimmung der Abstände zwischen Gelenk 2 und Gelenk 4. Dies wird deshalb
 * gemacht, weil sich die Gelenke 2, 3 und 4 alle entlang der selben
 * Koordinatenachse bewegen und nach Gelenk 4 schon der Endeffektor folgt.
 * Da dieser gemäß dem Winkel alpha zum Zielpunkt ausgerichtet sein muß,
 * reicht die Entfernung von Gelenk 2 zu Gelenk 4 um die Winkelstellungen
 * von diesen Gelenken zu berechnen.
 */
	xx = position[0] + sin(W[0][0])*cos(position[3])*laenge[3];   
	yy = position[1] - cos(W[0][0])*cos(position[3])*laenge[3];
	zz = position[2] + sin(position[3])*laenge[3];         
	fd = sqrt(xx*xx + yy*yy); 

// Nachfolgende Formeln entstammen der Buch "Introduction to robotics" und	
// wurde mittels Mathematischer Formelsammlung auf diesen Manipulatorarm
// abgespeckt und angepasst. 

	// Halber Umfang des Dreiecks: Gelenk 2, Gelenk 3, Gelenk 4
	hilf = 1.0/2.0*(laenge[1]+laenge[2]+sqrt(fd*fd+zz*zz));

	// Inkreisradius vom Gelenk 2 zum Gelenk 4
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

	// Winkelstellung von Gelenk 2 (Segment 2) gegenüber der Winkelstellung
	// des Inkreisradius
	theta_hilf_1 = 2.0*atan2(inkreisradius,(hilf-laenge[1]));

	// Winkelstellung von Gelenk 3 (Segment 3) gegenüber der Winkelstellung
	// des Inkreisradius
	theta_hilf_2 = 2.0*atan2(inkreisradius,(hilf-laenge[2]));

	// Winkelstellung des Inkreisradius
	theta_hilf_3 = atan2(zz,fd);


/* Da es für jede Variante des Gelenkes 1 theoretisch 2 Lösungen für die
 * Gelenkekombination 2,3,4 gibt und es theoretisch stehts zwei mögliche
 * Lösungen für das Gelenk 1 gibt, gibt es für die Gelenkekombination 2,3,4
 * somit insgesamt 4 mögliche Lösungen. Welche jetzt nachfolgend betrachtet 
 * werden. Die Winkelstellung des Gelenkes 4 ergibt sich dabei stehts aus
 * den beiden Gelenkwinkeln der Gelenke 2 und 3 und dem Zielwinkel alpha.
 */

	// Bestimmung von W2, W3, W4 (Obere Variante) bei W11
	W[1][0] = -(theta_hilf_3 + theta_hilf_2);
	W[2][0] = (-(theta_hilf_3 - theta_hilf_1)) - W[1][0];
	W[3][0] = fmod((position[3] - W[1][0] - W[2][0]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Untere Variante) bei W11
	W[1][1] = -(theta_hilf_3 - theta_hilf_2);
	W[2][1] = (-(theta_hilf_3 + theta_hilf_1)) - W[1][1];
	W[3][1] = fmod((position[3] - W[1][1] - W[2][1]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Obere Variante) bei W12
	W[1][2] = -(M_PI + (-(theta_hilf_3 + theta_hilf_2)));
	W[2][2] = W[2][1];
	W[3][2] = fmod((M_PI + position[3] - W[1][2] - W[2][1]),(2.0 * M_PI));

	// Bestimmung von W2, W3, W4 (Untere Variante) bei W12
	W[1][3] = -(M_PI + (-(theta_hilf_3 - theta_hilf_2)));
	W[2][3] = W[2][0];
	W[3][3] = fmod((M_PI + position[3] - W[1][3] - W[2][0]),(2.0 * M_PI));

	// Umwandlung der Winkel von Grad/Pi in Grad
	for(i=0; i<6; i++)
	{
		for(j=0; j<4; j++)
		{
			W[i][j] = W[i][j] * 180 / M_PI;
		}
	}

	// Anpassung der Winkelmatrix, so dass alle Winkel zwischen +/- 180°
	// liegen
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

	// Übergabe der Winkelstellungen an die Rückgabevektoren
	for (i=0; i<6; i++)
	{
		W1[i] = W[i][0];
		W2[i] = W[i][1];
		W3[i] = W[i][2];
		W4[i] = W[i][3];
	}

	// Wenn zwischen den Gelenken 2 und 4 eine negative Entfernung liegt,
	// so müssen die Winkelmatrizen noch nachträglich darauf angepasst 
	// werden.
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
}
