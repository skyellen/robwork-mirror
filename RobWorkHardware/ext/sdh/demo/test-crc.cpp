//======================================================================
/*!
## \file
#  \section sdhlibrary_cpp_test_crc_cpp_general General file information
#
#    \author   Dirk Osswald
#    \date     2008-06-12
#
#  \brief
#    Simple script to test CRC. See online help ("-h"
#    or "--help") for available options.
#
#  \section sdhlibrary_cpp_test_crc_cpp_copyright Copyright
#
#  - Copyright (c) 2008 SCHUNK GmbH & Co. KG
#
#  <HR>
#  \internal
#
#    \subsection sdhlibrary_cpp_test_crc_cpp_details SVN related, detailed file specific information:
#      $LastChangedBy: Osswald2 $
#      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
#      \par SVN file revision:
#        $Id: test-crc.cpp 3659 2008-10-08 08:48:38Z Osswald2 $
#
#  \subsection sdhlibrary_cpp_test_crc_cpp_changelog Changelog of this file:
#      \include test-crc.cpp.log
#
*/
//======================================================================

#include "sdh/sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

//#include <getopt.h>
//#include <assert.h>

#include <iostream>
#include <vector>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/crc.h"
#include "sdh/basisdef.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------




int main( int argc, char **argv )
{
    cCRC_DSACON32m crc;

    SDH_ASSERT_TYPESIZES();


    int i;
    for ( i = 1; i < argc; i++ )
    {
        int byte;
        sscanf( argv[i], "%d", &byte );


        crc.AddByte( byte );
        cout << "Added " << byte << " = 0x" << hex << byte << "\n";
    }

    cout << "crc = " << crc.GetCRC() << " = 0x" << hex << crc.GetCRC() << "\n";
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
