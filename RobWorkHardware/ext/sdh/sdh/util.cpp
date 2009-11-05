//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_util_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of auxilliary utility functions for SDHLibrary-CPP.

  \section sdhlibrary_cpp_util_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_util_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-08-08 19:05:54 +0200 (Fr, 08 Aug 2008) $
      \par SVN file revision:
        $Id: util.cpp 3444 2008-08-08 17:05:54Z Osswald2 $

  \subsection sdhlibrary_cpp_util_cpp_changelog Changelog of this file:
      \include util.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <math.h>
#include <time.h>

#include <iostream>
#if SDH_USE_VCC
# include <windows.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "util.h"
#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

NAMESPACE_SDH_START


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

bool InIndex( int v, int max )
{
    return 0 <= v  &&  v < max;
}
//-----------------------------------------------------------------

bool InRange( double v, double min, double max )
{
    return min <= v  &&  v <= max;
}
//-----------------------------------------------------------------

bool InRange( int n, double const* v, double const* min, double const* max )
{
    for ( int i=0; i < n; i++ )
    {
        if (! InRange( v[i], min[i], max[i] ))
            return false;
    }
    return true;
}
//-----------------------------------------------------------------

double ToRange( double v, double min, double max )
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}
//-----------------------------------------------------------------

void ToRange( int n, double* v, double const* min, double const* max )
{
    for ( int i=0; i < n; i++ )
    {
        v[i] = ToRange( v[i], min[i], max[i] );
    }
}
//-----------------------------------------------------------------

void ToRange( std::vector<double>& v, std::vector<double> const& min, std::vector<double> const& max )
{
    ToRange( int(v.size()), &(v[0]), &(min[0]), &(max[0]) );
}
//-----------------------------------------------------------------

double Approx( double a, double b, double eps )
{
    return fabs( a - b ) < eps;
}
//-----------------------------------------------------------------

bool Approx( int n, double* a, double* b, double* eps )
{
    for ( int i=0; i < n; i++ )
    {
        if (! Approx( a[i], b[i], eps[i] ))
            return false;
    }
    return true;
}
//-----------------------------------------------------------------

double DegToRad( double d )
{
    return d*M_PI/180.0;
}
//-----------------------------------------------------------------

double RadToDeg( double r )
{
    return r*180.0/M_PI;
}
//-----------------------------------------------------------------

void SleepSec( double t )
{
#if SDH_USE_VCC
    ::Sleep( static_cast<int>(1000.0*t) );
#else
    timespec sleeptime;
    sleeptime.tv_sec  = (time_t) floor( t );
    sleeptime.tv_nsec = (long)   ((t - floor( t )) * 1E9);

    ////std::cout << "Sleeping for " << sleeptime.tv_sec << "s and " << sleeptime.tv_nsec << "ns\n";

    nanosleep( &sleeptime, NULL );
#endif
}

NAMESPACE_SDH_END


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================
