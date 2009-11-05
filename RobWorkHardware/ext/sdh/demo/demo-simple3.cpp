//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_simple_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple C++ programm to make the SDH move. With non-sequential call of move and WaitAxis.

       This code contains only the very basicst use of the features
       provided by the SDHLibrary-CPP. For more sophisticated
       applications see the other demo-*.cpp programms, or of course
       the html/pdf documentation.

     \section sdhlibrary_cpp_sdh_demo_simple_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_simple_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
         \par SVN file revision:
           $Id: demo-simple3.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_simple_changelog Changelog of this file:
         \include demo-simple3.cpp.log

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
    \anchor sdhlibrary_cpp_demo_simple3_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Move axes 1,2 and 3 to a specific point.\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: demo-simple3.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_simple3_cpp_vars
//  @}



int main( int argc, char** argv )
{
    SDH_ASSERT_TYPESIZES();

    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "demo-simple3", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );
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

        // Open communication to the SDH device via default serial port 0 == "COM1"
        if ( options.use_can )
            hand.OpenCAN_ESD( options.net,            // ESD CAN net number
                              options.can_baudrate,   // CAN baudrate in bit/s
                              options.timeout,        // timeout in s
                              options.id_read,        // CAN ID used for reading
                              options.id_write );     // CAN ID used for writing
        else
            // Open communication to the SDH device via default serial port 0 == "COM1"
            hand.OpenRS232( options.port, options.rs232_baudrate, options.timeout );



        // Set a new target pose for axis 1,2 and 3
        std::vector<int> axes123;
        axes123.push_back( 1 );
        axes123.push_back( 2 );
        axes123.push_back( 3 );

        std::vector<double> angles123;
        angles123.push_back( -20.0 );
        angles123.push_back( -30.0 );
        angles123.push_back( -40.0 );

        hand.SetAxisTargetAngle( axes123, angles123 );

        // Move axes there non sequentially:
        hand.MoveAxis( axes123, false );

        // The last call returned immediately so we now have time to
        // do something else while the hand is moving:

        // ... insert any calculation here ...

        // Before doing something else with the hand make shure the
        // selected axes have finished the last movement:
        hand.WaitAxis( axes123 );


        // go back home (all angles to 0.0):
        hand.SetAxisTargetAngle( hand.All, 0.0 );

        // Move all axes there non sequentially:
        hand.MoveAxis( hand.All, false );

        // ... insert any other calculation here ...

        // Wait until all axes are there, with a timeout of 10s:
        hand.WaitAxis( hand.All, 10.0 );


        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (cSDHLibraryException* e)
    {
        std::cerr << "demo-simple3 main(): An exception was caught: " << e->what() << "\n";
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
