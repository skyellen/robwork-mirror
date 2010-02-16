//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!
   \file
   \section sdhlibrary_cpp_demo_dsa_cpp_general General file information

     \author   Dirk Osswald
     \date     2008-06-12

   \brief
     Simple program to test class cDSA.
     See \ref demo_dsa__help__ "__help__" and online help ("-h" or "--help") for available options.

   \section sdhlibrary_cpp_demo_dsa_cpp_copyright Copyright

   - Copyright (c) 2008 SCHUNK GmbH & Co. KG

   <HR>
   \internal

     \subsection sdhlibrary_cpp_demo_dsa_cpp_details SVN related, detailed file specific information:
       $LastChangedBy: Osswald2 $
       $LastChangedDate: 2009-11-02 09:20:24 +0100 (Mo, 02 Nov 2009) $
       \par SVN file revision:
         $Id: demo-dsa.cpp 4932 2009-11-02 08:20:24Z Osswald2 $

   \subsection sdhlibrary_cpp_demo_dsa_cpp_changelog Changelog of this file:
       \include demo-dsa.cpp.log

*/
/*!
  @}
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
  \anchor sdhlibrary_cpp_demo_dsa_cpp_vars
  \name   Some informative variables

  Some definitions that describe the demo program

  @{
*/
//! \anchor demo_dsa__help__
char const* __help__      =
    "Simple demo to test cDSA class of SDHLibrary-cpp.\n"
    "\n"
    "Remarks:\n"
    "- You must specify at least one of these options to see some output:\n"
    "  -f | --fullframe  \n"
    "  -r | --resulting\n"
    "  -c | --controllerinfo \n"
    "  -s | --sensorinfoinfo\n"
    "  -m | --matrixinfo=N\n"
    "  \n"
    "- Example usage:\n"
    "  - Read a single full frame from tactile sensors connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f\n"
    "     \n"
    "  - Read full frames continuously from tactile sensors connected to\n"
    "    port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -f -r 1\n"
    "     \n"
    "  - Read a single full frame from tactile sensors connected to USB\n"
    "    to RS232 converter 0:\n"
    "    > demo-dsa --dsa_rs_device=/dev/ttyUSB0 -f \n"
    "     \n"
    "  - Read the sensor, controller, matrix 0 infos \n"
    "    from tactile sensors connected to port 3 = COM4:\n"
    "    > demo-dsa --dsaport=3 -s -c -m 0 \n"
    "    \n"
    "  - Get the version info of both the joint controllers and the tactile \n"
    "    sensor firmware from an SDH connected to \n"
    "    - port 2 = COM3 (joint controllers) and \n"
    "    - port 3 = COM4 (tactile sensor controller) \n"
    "    > demo-dsa -p 2 --dsaport=3 -v\n"
    "\n"
    "- Known bugs:"
    "  - see the bug description for \"cDSAException: Checksum Error on Windows\n"
    "    console\" in the Related Pages->Bug List section of the doxygen\n"
    "    documentation\n"
    ;

char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-dsa.cpp 4932 2009-11-02 08:20:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2008 SCHUNK GmbH & Co. KG";

char const* usage =
  "usage: demo-dsa [options]\n"
  ;

//  end of doxygen name group sdhlibrary_cpp_demo_dsa_cpp_vars
//  @}
//----------------------------------------------------------------------



int main( int argc, char **argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options( "general sdhcom_common dsacom dsaother" );

    options.Parse( argc, argv, __help__, "demo-dsa", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

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

    //---------------------
    // start actual processing:
    try
    {
        cDSA ts = cDSA( options.debug_level, options.dsaport, options.dsa_rs_device );


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
        if ( options.fullframe || options.framerate > 0 )
            ts.SetFramerate( 1, options.do_RLE ); // remark: any value > 0 will make the remote DSACON32m in the SDH send data with the highest possible datarate
        //---------------------


        //-----------
        // start periodic or one time processing of full frames if requested:
        bool finished = !options.fullframe;
        while (! finished )
        {
            //-----------
            if ( options.fullframe )
            {
                ts.UpdateFrame();
                cout << ts;
            }
            //-----------

            //-----------
            if ( options.framerate <= 0 )
                finished = true;
        }
        //---------------------

        ts.Close();
    }
    catch ( cSDHLibraryException* e )
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
