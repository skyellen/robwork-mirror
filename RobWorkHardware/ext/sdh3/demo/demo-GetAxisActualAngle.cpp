//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_getaxisactualangle_general General file information

       \author   Dirk Osswald
       \date     2007-03-07

     \brief
       Print measured actual axis angles of an attached %SDH. (C++ demo application using the SDHLibrary-CPP library.)
       See \ref demo_getaxisactualangle__help__ "__help__" and online help ("-h" or "--help") for available options.

     \section sdhlibrary_cpp_sdh_demo_getaxisactualangle_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_getaxisactualangle_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2010-12-02 14:59:45 +0100 (Do, 02 Dez 2010) $
         \par SVN file revision:
           $Id: demo-GetAxisActualAngle.cpp 6265 2010-12-02 13:59:45Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_getaxisactualangle_changelog Changelog of this file:
         \include demo-GetAxisActualAngle.cpp.log

*/
/*!
  @}
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

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
    \anchor sdhlibrary_cpp_demo_getaxisactualangle_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_getaxisactualangle__help__
char const* __help__      =
    "Print measured actual axis angles of SDH.\n"
    "(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "- Example usage:\n"
    "  - Print actual angles of an SDH connected to port 2 = COM3 once:\n"
    "    > demo-GetAxisActualAngle -p 2\n"
    "    \n"
    "  - Print actual angles of an SDH connected to port 2 = COM3 every 500ms:\n"
    "    > demo-GetAxisActualAngle -p 2 -t 0.5\n"
    "     \n"
    "  - Print actual angles of an SDH connected to USB to RS232 converter 0 once:\n"
    "    > demo-GetAxisActualAngle --sdh_rs_device=/dev/ttyUSB0 \n"
    "     \n"
    "  - Get the version info of an SDH connected to port 2 = COM3 \n"
    "    > demo-GetAxisActualAngle --port=2 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-GetAxisActualAngle.cpp 6265 2010-12-02 13:59:45Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_getaxisactualangle_cpp_vars
//  @}

char const* usage =
  "usage: demo-GetAxisActualAngle [options]\n"
  ;


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( SDHUSAGE_DEFAULT " sdhother" );

    options.Parse( argc, argv, __help__, "demo-GetAxisActualAngle", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
    //
    //---------------------

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
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";


        cdbg << "Caption:\n";
        if (options.period)
            cdbg << "  times are reported in seconds\n";

        cdbg << "  angles are reported in " << hand.uc_angle->GetName() << "[" << hand.uc_angle->GetSymbol() << "]\n";


        //??? a second try block to catch keyboard interrupts
        //try:
#if SDH_USE_VCC
        double elapsed = 0.0;
#else
        cSimpleTime start;
#endif

        while (true)
        {
            vector<double> angles = hand.GetAxisActualAngle( hand.all_axes );

            if (options.period > 0)
            {
                // print time only if reporting periodically
#if SDH_USE_VCC
                cout << elapsed << " ";
                elapsed += options.period;
#else
                cout << start.Elapsed() << " ";
#endif
            }

            for ( vector<double>::const_iterator ai = angles.begin();
                  ai != angles.end();
                  ai++ )
                cout << *ai << " ";

            cout << "\n";
            cout.flush();

            if (options.period <= 0)
                break;

            SleepSec( options.period );
        }

        hand.Close();
        cdbg << "Successfully disabled controllers of SDH and closed connection\n";
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "demo-GetAxisActualAngle main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "caught unexpected exception!\n";
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
