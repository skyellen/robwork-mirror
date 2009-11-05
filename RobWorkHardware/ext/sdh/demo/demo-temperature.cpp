//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_temperature_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Print measured temperatures of SDH. (C++ demo application using the SDHLibrary-CPP library.)

     \section sdhlibrary_cpp_sdh_demo_temperature_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_temperature_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
         \par SVN file revision:
           $Id: demo-temperature.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_temperature_changelog Changelog of this file:
         \include demo-temperature.cpp.log

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
    \anchor sdhlibrary_cpp_demo_temperature_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Print measured temperatures of SDH.\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-temperature.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_temperature_cpp_vars
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

    options.Parse( argc, argv, __help__, "demo-temperature", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
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


        cdbg << "Caption:\n";
        if (options.period)
            cdbg << "  times are reported in seconds\n";

        cdbg << "  temperatures are reported in " << hand.uc_temperature->GetName()
             << " [" << hand.uc_temperature->GetSymbol() << "]\n";


        //??? a second try block to catch keyboard interrupts
        //try:
#if SDH_USE_VCC
        double elapsed = 0.0;
#else
        cSimpleTime start;
#endif


        while (true)
        {
            vector<double> temps = hand.GetTemperature( hand.all_temperature_sensors );

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

            for ( vector<double>::const_iterator vi = temps.begin();
                  vi != temps.end();
                  vi++ )
                cout << *vi << " ";

            //print "%6.*f" % (hand.uc_temperature.decimal_places, v),

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
        cerr << "demo-temperature main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
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
