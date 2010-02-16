//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_serialbase_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-20

  \brief
    Implementation of class #SDH::cSerialBase, a virtual base class to access serial interfaces like RS232 or CAN.

  \section sdhlibrary_cpp_serialbase_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_serialbase_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-12-04 17:05:53 +0100 (Fr, 04 Dez 2009) $
      \par SVN file revision:
        $Id: serialbase.cpp 5022 2009-12-04 16:05:53Z Osswald2 $

  \subsection sdhlibrary_cpp_serialbase_cpp_changelog Changelog of this file:
      \include serialbase.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <termios.h>
# include <unistd.h>
# include <sys/select.h>
# include <sys/ioctl.h>
#endif
#include <errno.h>
//#include <string.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <cstring>    // needed in gcc-4.3 for prototypes like strcmp according to http://gcc.gnu.org/gcc-4.3/porting_to.html

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "serialbase.h"
#include "simpletime.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;
USING_NAMESPACE_SDH

char* cSerialBase::readline( char* line, int size, char const* eol, bool return_on_less_data )
    throw(cSerialBaseException*)
{
    line[0] = '\0';
    int len = 0;
    int n;
    char c;

    if (ungetch_valid)
    {
        line[len++] = ungetch;
        ungetch_valid = false;
    }

    ////cSimpleTime start;
    long timeout_us = (long) (timeout*1000000.0);

    while (true)
    {
        //    n = ::read(fd, line + len, 1);
        n = Read( line + len, 1, timeout_us, return_on_less_data );

        if (n>0)
        {
            c = line[ len ];
            len += n;

            //cout << "read '" << c << "' from device\n"; cout.flush();

            if (strchr( eol, c ) != NULL)
                break;


            if (size > 0  &&  len >= size)
                break;
        }
        else
        {
            throw new cSerialBaseException( cMsg( "Timeout while reading line from device (timeout_us=%ld line=\"%s\")", timeout_us, line ) );
        }
    }

    ////cerr << "reading line took " << start.Elapsed() << "s\n";

    // read on as long as eol chars are available => will greadily read sequences of eol as ONE line
    timeout_us = 0;

    while (true)
    {
        //  n = ::read( fd, line+len, 1 );
        n = Read( line + len, 1, timeout_us, true );

        if (n>0)
        {
            c = line[ len ];
            len += n;

            if (strchr( eol, c ) != NULL)
            {
                // keep c in line
            }
            else
            {
                //cerr << "cSerialBase::readline() ungetting " << (int)c << "=" << c <<"\n";
                // not an EOL char -> first char of next line
                ungetch = c; // remember for next call
                ungetch_valid = true;
                len -= 1;
                line[ len ] = '\0';
                break;
            }
        }
        else
        {
            line[ len ] = '\0';
            break;
        }
    }
    ////cerr << "reading eol took " << start.Elapsed() << "s\n";

    return line;
}
//----------------------------------------------------------------------



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
