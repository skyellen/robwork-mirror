//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_sdh_demo_ref_general General file information

       \author   Dirk Osswald
       \date     2007-01-18

     \brief
       Very simple C++ programm to make the SDH move

       This code contains only the very basicst use of the features
       provided by the SDHLibrary-CPP. For more sophisticated
       applications see the other demo-*.cpp programms, or of course
       the html/pdf documentation.

     \section sdhlibrary_cpp_sdh_demo_ref_copyright Copyright

     Copyright (c) 2007 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_sdh_demo_ref_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
         \par SVN file revision:
           $Id: demo-ref.cpp 3659 2008-10-08 08:48:38Z Osswald2 $

     \subsection sdhlibrary_cpp_sdh_demo_ref_changelog Changelog of this file:
         \include demo-ref.cpp.log

*/
//======================================================================

#include <iostream>
#include <vector>


// Include the cSDH interface
#include "sdh/sdh.h"

#include "sdh/util.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdh/basisdef.h"

USING_NAMESPACE_SDH


int main()
{
    bool   use_radians    = false;
    bool   use_fahrenheit = false;
    int    debug_level    = 0;     // the higher the value the more verbose the messages
    int    port           = 0;     // RS232 port number: 0 = "COM1" = "/dev/ttyS0"
    double timeout        = -1.0;  // timeout for RS232 communication in seconds, -1 is "no timeout", which is what we need.
    int    iFinger        = 1;     // The index of the finger to move
    unsigned long rs232_baudrate = 115200;

    SDH_ASSERT_TYPESIZES();


    try
    {
        // Create an instance "hand" of the class cSDH:
        cSDH hand( use_radians, use_fahrenheit, debug_level );

        // Open communication to the SDH device via default serial port 0 == "COM1"
        hand.OpenRS232( port, rs232_baudrate, timeout );


        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //
        // SDH with serial number 003 is missing the absolute
        // encoders in some axes!!!
        //
        // If you did not save the last position before turning off
        // the power then these axes must be referenced manually before
        // you can actually use them! This is done as first step below:
        //
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        double value = 2;     // 2 is "outward direction", this should
                              // be save.
                              // (1 would be "inward direction" which
                              // is likely to produce collisions)

        hand.comm_interface.ref( 3, &value );
        hand.comm_interface.ref( 4, &value );


        // Now perform some action:
        //   get the current actual axis angles of finger iFinger:
        std::vector<double> faa = hand.GetFingerActualAngle( iFinger );
        faa[1] = 0.0; // overwrite proximal
        faa[2] = 0.0; // and distal joint

        //   modify faa by decrementing the proximal and the distal axis angles
        //   (make a copy fta of faa and modify that to keep actual pose available)
        std::vector<double> fta = faa;

        fta[1] -= 10.0;
        fta[2] -= 10.0;

        //   keep fta in range too:
        ToRange( fta, hand.GetFingerMinAngle( iFinger ), hand.GetFingerMaxAngle( iFinger ) );

        std::cout << "Moving finger " << iFinger << " between faa=" << faa << " and fta=" << fta << "\n";

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
    catch (cSDHLibraryException* e)
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
