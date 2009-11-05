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
      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
      \par SVN file revision:
        $Id: serialbase.cpp 3659 2008-10-08 08:48:38Z Osswald2 $

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

char* cSerialBase::readline( char* line, int size, char * eol, bool return_on_less_data )
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
            throw new cSerialBaseException( cMsg( "Timeout while reading line from device" ) );
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
                // not an EOL char -> first char of next line
                ungetch = c; // remember for next call
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
