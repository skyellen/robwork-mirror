//======================================================================
/*!
## \file
#  \section sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_general General file information
#
#    \author   Dirk Osswald
#    \date     2008-10-02
#
#  \brief
#    Simple program to test access via namespace SDH::cDSA. See online help ("-h"
#    or "--help") for available options.
#
#  \section sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_copyright Copyright
#
#  - Copyright (c) 2008 SCHUNK GmbH & Co. KG
#
#  <HR>
#  \internal
#
#    \subsection sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_details SVN related, detailed file specific information:
#      $LastChangedBy: Osswald2 $
#      $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
#      \par SVN file revision:
#        $Id: test-sdhnamespace-dsa.cpp 3686 2008-10-13 15:07:24Z Osswald2 $
#
#  \subsection sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_changelog Changelog of this file:
#      \include test-sdhnamespace-dsa.cpp.log
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

#include "sdh/sdh.h"
#include "sdh/dsa.h"
#include "dsaoptions.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

// we want to access the SDH namespace explicitly here!
//USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------

/*!
  \anchor sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_vars
  \name   Some informative variables

  Some definitions that describe the demo program

  @{
*/
char const* __help__      =
    "Simple demo to test use of SDH namespace on cDSA class of SDHLibrary-cpp.\n"
    "\n"
    "Remarks:\n"
    "- You must specify at least one of the following options to to see\n"
    "  some output: --fullframe, --sensorinfo, --controllerinfo, --matrixinfo\n"
    "- Use --framerate=5 --fullframe to print a full frame 5 times per second. \n"
    ;

char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: test-sdhnamespace-dsa.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2008 SCHUNK GmbH & Co. KG";

char const* usage =
  "usage: test-sdhnamespace-dsa [options]\n"
  ;

//  end of doxygen name group sdhlibrary_cpp_test_sdhnamespace_dsa_cpp_vars
//  @}
//----------------------------------------------------------------------



int main( int argc, char **argv )
{
    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cDSAOptions options;

    options.Parse( argc, argv, __help__, "demo-dsa", __version__, SDH::cSDH::GetLibraryName(), SDH::cSDH::GetLibraryRelease() );

    //
    //---------------------

    //---------------------
    // initialize debug message printing:
    SDH::cDBG cdbg( options.debug_level>0, "red", options.debuglog );
    SDH::g_sdh_debug_log = options.debuglog;

    cdbg << "Debug messages of demo-program are printed like this.\n";

    //# reduce debug level for subsystems
    options.debug_level-=1;
    //---------------------

    //---------------------
    // start actual processing:
    try
    {
        SDH::cDSA ts = SDH::cDSA( options.debug_level, options.dsaport );


        //---------------------
        // print requested info:
        if ( options.controllerinfo )
        {
            cout << "Controller Info:\n";
            cout << ts.GetControllerInfo();
        }

        if ( options.sensorinfo )
        {
            cout << "Sensor Info:\n";
            cout << ts.GetSensorInfo();
        }

        if ( options.matrixinfo != -1 )
        {
            cout << "Matrix Info " << options.matrixinfo << ":\n";
            cout << ts.GetMatrixInfo( options.matrixinfo );
        }
        //---------------------

        //---------------------
        // Make remote tactile sensor controller send data periodically if requested:
        if ( options.framerate > 0 )
            ts.SetFramerate( options.framerate, options.do_RLE );
        //---------------------


        //-----------
        // start periodic or one time processing of full frames:
        bool finished = false;
        while (! finished )
        {
            //-----------
            if ( options.fullframe )
            {
                if ( options.framerate <= 0 )
                    ts.SetFramerate( 1, options.do_RLE );

                ts.UpdateFrame();

                if ( options.framerate <= 0 )
                    ts.SetFramerate( 0, options.do_RLE );

                cout << ts;
            }
            //-----------

            //-----------
            if ( options.framerate < 0 )
                finished = true;
        }
        //---------------------

        ts.Close();
    }
    catch ( SDH::cSDHLibraryException* e )
    {
        cerr << "\ndemo-dsa main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "\ncaught unknown exception, giving up\n";
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
