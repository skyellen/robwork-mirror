//======================================================================
/*!
  \addtogroup sdh_library_cpp_demo_programs_group
  @{
*/
/*!

    \file
     \section sdhlibrary_cpp_sdh_demo_griphand_general General file information

       \author   Dirk Osswald
       \date     2009-12-04

     \brief
       Very simple demonstration program using the SDHLibrary-CPP: Demonstrate the use of the GripHand command
       See \ref demo_griphand__help__ "__help__" and online help ("-h" or "--help") for available options.

       \warning The cSDH::GripHand() function is somewhat problematic (not interruptible), see its documentation.
     \section sdhlibrary_cpp_sdh_demo_griphand_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_griphand_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2010-12-02 14:59:45 +0100 (Do, 02 Dez 2010) $
         \par SVN file revision:
           $Id: demo-griphand.cpp 6265 2010-12-02 13:59:45Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_griphand_changelog Changelog of this file:
         \include demo-griphand.cpp.log

*/
/*!
  @}
*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"
#include "sdhoptions.h"

USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_griphand_cpp_vars
    \name   Some informative variables

    @{
*/
//! \anchor demo_griphand__help__
char const* __help__      =
    "Demonstrate the use of the GripHand command.\n"
    "(C++ demo application using the SDHLibrary-CPP library.)\n"
    "\n"
    "- Example usage:\n"
    "  - Make SDH connected to port 2 = COM3 move:\n"
    "    > demo-griphand -p 2\n"
    "     \n"
    "  - Make SDH connected to USB to RS232 converter 0 move:\n"
    "    > demo-griphand --sdh_rs_device=/dev/ttyUSB0 \n"
    "     \n"
    "  - Get the version info of an SDH connected to port 2 = COM3 \n"
    "    > demo-griphand --port=2 -v\n";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-griphand.cpp 6265 2010-12-02 13:59:45Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_griphand_cpp_vars
//  @}

char const* usage =
  "usage: demo-griphand [options]\n"
  ;


int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-griphand", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
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
        // Create an instance "hand" of the class cSDH:
        cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );
        cdbg << "Successfully created cSDH instance\n";

        // Open configured communication to the SDH device
        options.OpenCommunication( hand );
        cdbg << "Successfully opened communication to SDH\n";

        // Switch to "pose" controller mode and set default velocities first:
        hand.SetController( hand.eCT_POSE );

        cSDH::eGraspId grasp_id = cSDH::eGID_CENTRICAL; // start with first valid grasp
        double close_ratio;
        double velocity = 40.0;

        while ( grasp_id < cSDH::eGID_DIMENSION )
        {
            // remark: the std::cout.flush() is needed to make the output appear in time
            // when compiled with VCC
            std::cout << "Performing GripHand() for grasp " << int(grasp_id) << "=\"" << hand.GetStringFromGraspId( grasp_id ) << "\"\n"; std::cout.flush();

            close_ratio = 0.0;
            std::cout << "   close_ratio = 0.0 (fully open)..."; std::cout.flush();
            hand.GripHand( grasp_id, close_ratio, velocity, true );
            std::cout << " finished\n"; std::cout.flush();
            SleepSec( 1.0 );

            close_ratio = 0.5;
            std::cout << "   close_ratio = 0.5 (half closed)..."; std::cout.flush();
            hand.GripHand( grasp_id, close_ratio, velocity, true );
            std::cout << " finished\n"; std::cout.flush();
            SleepSec( 1.0 );

            close_ratio = 1.0;
            std::cout << "   close_ratio = 1.0 (fully closed)..."; std::cout.flush();
            hand.GripHand( grasp_id, close_ratio, velocity, true );
            std::cout << " finished\n"; std::cout.flush();
            SleepSec( 1.0 );


            close_ratio = 0.0;
            std::cout << "   reopening..."; std::cout.flush();
            hand.GripHand( grasp_id, close_ratio, velocity, true );
            std::cout << " finished\n"; std::cout.flush();

            grasp_id = (cSDH::eGraspId)( grasp_id + 1 );
        }

        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-griphand main(): An exception was caught: " << e->what() << "\n";
        delete e;
    }
}
//----------------------------------------------------------------------


//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored)
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================]
