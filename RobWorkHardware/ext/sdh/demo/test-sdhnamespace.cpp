//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_test_namespace_general General file information

       \author   Dirk Osswald
       \date     2008-10-01

     \brief
       Very simple C++ programm that explicitly uses the SDH namespace.
       if SDH_USE_NAMESPACE in sdhlibrary_settings is 0 then this will not compile!


     \section sdhlibrary_cpp_sdh_test_namespace_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_test_namespace_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
         \par SVN file revision:
           $Id: test-sdhnamespace.cpp 3659 2008-10-08 08:48:38Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_test_namespace_changelog Changelog of this file:
         \include demo-simple.cpp.log

*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#include "sdh/sdh.h"
#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdhoptions.h"

using namespace std;

// we want to access the SDH namespace explicitly here!
//USING_NAMESPACE_SDH

/*!
    \anchor sdhlibrary_cpp_test_namespace_cpp_vars
    \name   Some informative variables

    @{
*/
char const* __help__      = "Test use of SDH namespace\n(C++ demo application using the SDHLibrary-CPP library.)";
char const* __author__    = "Dirk Osswald: dirk.osswald@de.schunk.com";
char const* __url__       = "http://www.schunk.com";
char const* __version__   = "$Id: test-sdhnamespace.cpp 3659 2008-10-08 08:48:38Z Osswald2 $";
char const* __copyright__ = "Copyright (c) 2007 SCHUNK GmbH & Co. KG";

//  end of doxygen name group sdhlibrary_cpp_test_namespace_cpp_vars
//  @}

char* usage =
  "usage: demo-simple [options]\n"
  ;


int main( int argc, char** argv )
{
    //---------------------
    // handle command line options: set defaults first then overwrite by parsing actual command line
    cSDHOptions options;

    options.Parse( argc, argv, __help__, "test-namespace", __version__, SDH::cSDH::GetLibraryName(), SDH::cSDH::GetLibraryRelease() );

    //
    //---------------------

    int    iFinger        = 0;     // The index of the finger to move

    try
    {
        // Create an instance "hand" of the class cSDH:
        SDH::cSDH hand( options.use_radians, options.use_fahrenheit, options.debug_level );

        if ( options.use_can )
            hand.OpenCAN_ESD( options.net,          // ESD CAN net number
                              options.can_baudrate, // CAN baudrate in bit/s
                              options.timeout,      // timeout in s
                              options.id_read,      // CAN ID used for reading
                              options.id_write );   // CAN ID used for writing
        else
            // Open communication to the SDH device via default serial port 0 == "COM1"
            hand.OpenRS232( options.port, options.rs232_baudrate, options.timeout );


        // Now perform some action:
        //   get the current actual axis angles of finger iFinger:
        std::vector<double> faa = hand.GetFingerActualAngle( iFinger );

        //   sometimes the actual angles are reported slightly out of range
        //   (Like -0.001 for axis 0 ). So limit the angles to the allowed range:
        SDH::ToRange( faa, hand.GetFingerMinAngle( iFinger ), hand.GetFingerMaxAngle( iFinger ) );

        //   modify faa by decrementing the proximal and the distal axis angles
        //   (make a copy fta of faa and modify that to keep actual pose available)
        std::vector<double> fta = faa;

        fta[1] -= 10.0;
        fta[2] -= 10.0;

        //   keep fta in range too:
        SDH::ToRange( fta, hand.GetFingerMinAngle( iFinger ), hand.GetFingerMaxAngle( iFinger ) );

        // we should use the using namespace here, since else the call to the
        // operator<< overloaded for std::vector<double> in our SDH:: namespace is really ugly:

        SDH::operator<<( SDH::operator<<( std::cout << "Moving finger " << iFinger << " between faa=", faa ) << " and fta=", fta ) << "\n";

        // The more intuitive approach does not work:
        // cout << faa ;                // obviously fails with "no match for >>operator<<<< in >>std::cout << faa<<", as expected
        // cout SDH::<< faa ;           // is a syntax error
        // cout SDH::operator<< faa ;   // is a syntax error
        // cout operator SDH::<< faa ;  // is a syntax error
        {
            using namespace SDH;
            std::cout << "Moving finger " << iFinger << " between faa=" << faa << " and fta=" << fta << "\n";
        }

        //   now move for 3 times between these two poses:
        for (int i=0; i<3; i++ )
        {
            // set a new target angles
            hand.SetFingerTargetAngle( iFinger, fta );

            // and make the finger move there:
            hand.MoveFinger( iFinger );


            // set a new target angles
            hand.SetFingerTargetAngle( iFinger, faa );

            // and make the finger move there:
            hand.MoveFinger( iFinger );

        }


        // Finally close connection to SDH again, this switches the axis controllers off
        hand.Close();
    }
    catch (SDH::cSDHLibraryException* e)
    {
        std::cerr << "demo-simple main(): An exception was caught: " << e->what() << "\n";
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
