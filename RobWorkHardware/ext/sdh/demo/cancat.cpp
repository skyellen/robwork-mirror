//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_cancat_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple C++ programm to make the SDH move

       This code contains only the very basicst use of the features
       provided by the SDHLibrary-CPP. For more sophisticated
       applications see the other demo-*.cpp programms, or of course
       the html/pdf documentation.

     \section sdhlibrary_cpp_sdh_cancat_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_cancat_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
         \par SVN file revision:
           $Id: cancat.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_cancat_changelog Changelog of this file:
         \include cancat.cpp.log

*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/dbg.h"
#include "sdh/canserial-esd.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdhoptions.h"

using namespace std;
USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_demo_simple_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Send data from command line via ESD CAN and display replies until CTRL-C is pressed.";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: cancat.cpp 3686 2008-10-13 15:07:24Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_demo_simple_cpp_vars
//  @}

char* usage =
  "usage: cancat [options]\n"
  ;


int main( int argc, char** argv )
{
    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.use_can = true;
    options.timeout = 0.0;

    options.Parse( argc, argv, __help__, "cancat", __version__, cSDH::GetLibraryName(), cSDH::GetLibraryRelease() );

    //
    //---------------------

    cDBG dbg( options.debug_level > 0, "red", options.debuglog );

    try
    {
        cCANSerial_ESD canserial( options.net, options.can_baudrate, options.timeout, options.id_read, options.id_write );

        canserial.Open();

        if ( optind < argc)
        {
            char buffer[ 512 ];
            buffer[0] = '\0';
            char* sep = "";
            while (optind < argc)
            {
                strncat( buffer, argv[ optind++ ], 511 );
                strncat( buffer, sep, 511 );
                sep = " ";
            }
            strncat( buffer, "\r\n", 511 );

            dbg << "Sending String \"" << buffer << "\" via CAN\n";


            canserial.write( buffer, strlen( buffer ) );
        }
        while ( 1 )
        {
            char reply;
            canserial.Read( &reply, 1, 0, false );
            printf( "%c", reply );
            fflush( stdout );
        }
    }
    catch ( cSDHLibraryException* e )
    {
        cerr << "cancat main(): Caught exception from SDHLibrary: " << e->what() << ". Giving up!\n";
        delete e;
    }
    catch (...)
    {
        cerr << "cancat main(): caught unexpected exception!\n";
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
