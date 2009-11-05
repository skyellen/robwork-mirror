//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_test_general General file information

       \author   Dirk Osswald
       \date     2007-03-07

     \brief
       Print measured actual axis angles of SDH. (C++ demo application using the SDHLibrary-CPP library.)

     \section sdhlibrary_cpp_sdh_demo_test_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_test_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
         \par SVN file revision:
           $Id: demo-test.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_test_changelog Changelog of this file:
         \include demo-test.cpp.log

*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <getopt.h>
#include <assert.h>

#include <iostream>
#include <vector>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/sdh.h"
#include "sdh/simpletime.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
    \anchor sdhlibrary_cpp_demo_getaxisactualangle_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Tries to connect to SDH, read actual angles and exits.\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-test.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_getaxisactualangle_cpp_vars
//  @}



//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-test", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
    //
    //----------------------------------------------------------------------

    //---------------------
    // initialize debug message printing:
    cDBG cdbg( options.debug_level > 0, "red", options.debuglog );
    g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of " << argv[0] << " are printed like this.\n";

    // reduce debug level for subsystems
    options.debug_level-=1;
    //---------------------

    try
    {
        // cSDH instance "hand" of the class cSDH according to the given options:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        if ( options.use_can )
            hand.OpenCAN_ESD( options.net, options.can_baudrate, options.timeout, options.id_read, options.id_write );
        else
            hand.OpenRS232( options.port, options.rs232_baudrate, options.timeout );
        cdbg << "Successfully opened communication to SDH\n";


        cdbg << "hand.IsOpen() = " << hand.IsOpen() << "\n";

        vector<double> angles = hand.GetAxisActualAngle( hand.all_axes );
        cdbg << "hand.GetAxisActualAngle( hand.all_axes ) = " << angles << "\n";


        hand.Close();
        cdbg << "Successfully disabled controllers of SDH and closed connection\n";
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "demo-test main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "demo-test main(): Caught some exception (should not happen)!\n";
    }

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

