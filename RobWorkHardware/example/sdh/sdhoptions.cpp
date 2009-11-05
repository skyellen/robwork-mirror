//======================================================================
/*!
    \file

       \author   Dirk Osswald
       \date     2008-05-05

     \brief
       Implementation of a class to parse common SDH related command line options

     \section sdhlibrary_cpp_demo_sdhoptions_cpp_copyright Copyright

     Copyright (c) 2008 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_demo_sdhoptions_cpp_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
         \par SVN file revision:
           $Id: sdhoptions.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

     \subsection sdhlibrary_cpp_demo_sdhoptions_cpp_changelog Changelog of this file:
         \include sdhoptions.cpp.log

*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <getopt.h>
#include <assert.h>

#include <iostream>
#include <fstream>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/sdh.h"
#include "sdh/sdhlibrary_settings.h"
#include "sdhoptions.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


static char* sdhoptions_usage =
  "options:\n"
  "  -h, --help            show this help message and exit\n"
  "  -p PORT, --port=PORT  use RS232 communication PORT to connec to to SDH\n"
  "                        instead of default 0='COM1'='/dev/ttyS0'.\n"
  "  -d LEVEL, --debug=LEVEL\n"
  "                        Print debug messages of level LEVEL or lower while\n"
  "                        execting the programm. Level 0 (default): No\n"
  "                        messages,  1: application-level messages, 2: cSDH-level\n"
  "                        messages, 3: cSDHSerial-level messages\n"
  "  -l LOGFILE, --debuglog=LOGFILE\n"
  "                        Redirect the printed debug messages to LOGFILE instead\n"
  "                        of standard error (default). If LOGFILE starts with\n"
  "                        '+' then the output will be appended to the file\n"
  "                        (without the leading '+'), else the file will be\n"
  "  -R, --radians         Use radians and radians per second for angles and\n"
  "                        angular velocities instead of default degrees and\n"
  "                        degrees per second\n"
  "  -F, --fahrenheit      Use degrees fahrenheit to report temperatures instead\n"
  "                        of default degrees celsius\n"
  "  -v, --version         Print the version (revision/release names of application,\n"
  "                        library and firmware then exit.\n"
  "  -t PERIOD, --period=PERIOD\n"
  "                        Time period of measurements in seconds. The default of\n"
  "                        '0' means: report once only. If set then the time\n"
  "                        since start of measurement is printed at beginning of\n"
  "                        every line\n"
  "  -T TIMEOUT, --timeout=TIMEOUT\n"
  "                        Timeout in seconds when waiting for data from SDH\n"
  "                        The default -1 means: wait forever.\n"
  "  -c, --can             use ESD CAN instead of RS232\n"
  "  -n NET, --net=NET     use ESD CAN NET for CAN communication, default=0\n"
  "  -b BAUDRATE, --baud=BAUDRATE\n"
  "                        use BAUDRATE (in bit/s) for communication.\n"
  "                        default is 1000000 Bit/s (1MBit/s) for CAN\n"
  "                        and 115200 Bit/s for RS232\n"
  "  -r ID_READ, --id_read=ID_READ\n"
  "                        use CAN ID ID_READ for receiving CAN messages (default: 43)\n"
  "  -w ID_WRITE, --id_write=ID_WRITE \n"
  "                        use CAN ID ID_WRITE for writing CAN messages (default: 42)\n"
  ;

static char* sdhoptions_short_options = "hp:d:RFvt:T:cn:b:r:w:l:";
static struct option sdhoptions_long_options[] =
{
  // name, has_arg, flag, val
  {"help",       0, 0, 'h'},
  {"port",       1, 0, 'p'},
  {"debug",      1, 0, 'd'},
  {"debuglog",   1, 0, 'l'},
  {"radians",    0, 0, 'R'},
  {"fahrenheit", 0, 0, 'F'},
  {"version",    0, 0, 'v'},
  {"period",     1, 0, 't'},
  {"timeout",    1, 0, 'T'},
  {"can",        0, 0, 'c'},
  {"net",        1, 0, 'n'},
  {"baud",       1, 0, 'b'},
  {"id_read",    1, 0, 'r'},
  {"id_write",   1, 0, 'w'},

  {0, 0, 0, 0}
};


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


cSDHOptions::cSDHOptions( void )
{
    usage = sdhoptions_usage;

    // set default options
    port           = 0;    // 0=/dev/ttyS0=COM1
    rs232_baudrate = 115200;
    debug_level    = 0;    // 0: no debug messages
    debuglog       = &cerr;
    use_radians    = false;
    use_fahrenheit = false;
    period         = 0.0;  // no period, read only once
    timeout        = -1.0; // no timeout, wait forever (which is what we need)
    use_can        = false;
    net            = 0;
    can_baudrate   = 1000000;
    id_read        = 43;
    id_write       = 42;
}

int cSDHOptions::Parse( int argc, char** argv,
                         char const* helptext, char const* progname, char const* version, char const* libname, char const* librelease )
{
    // parse options from command line
    unsigned long baudrate = 0;
    bool do_print_version = false;
    int option_index = 0;
    int rc;

    while (1)
    {
        int c;
        c = getopt_long( argc, argv,
                         sdhoptions_short_options, sdhoptions_long_options,
                         &option_index );
        if (c == -1)
            break;

        switch (c)
        {
        case 'h':
            cout << helptext << "\n\nusage: " << progname << " [options]\n" << usage << "\n";
            exit(0);

        case 'p':
            rc = sscanf( optarg, "%d", &port );
            assert( rc == 1 );
            break;

        case 'd':
            rc = sscanf( optarg, "%d", &debug_level );
            assert( rc == 1 );
            break;

        case 'l':
        {
            ios_base::openmode mode = ios_base::out | ios_base::trunc;
            if ( optarg[0] == '+' )
            {
                mode = ios_base::app;
                optarg = optarg+1;
            }
            debuglog = new ofstream( optarg, mode );
            assert( debuglog != NULL );
            assert( ! debuglog->fail() );
            break;
        }
        case 'R':
            use_radians = true;
            break;

        case 'F':
            use_fahrenheit = false;
            break;

        case 'v':
            do_print_version = true; // defer actual printing until all options are parsed (which might change the communication to use)
            break;

        case 't':
            rc = sscanf( optarg, "%lf", &period );
            assert( rc == 1 );
            break;

        case 'T':
            rc = sscanf( optarg, "%lf", &timeout );
            assert( rc == 1 );
            break;

        case 'c':
            use_can = true;
            break;

        case 'n':
            rc = sscanf( optarg, "%d", &net );
            assert( rc == 1 );
            break;

        case 'b':
            rc = sscanf( optarg, "%lu", &baudrate ); // store in intermediate variable since -b might be specified before --can
            assert( rc == 1 );
            break;

        case 'r':
            if ( !strncmp( optarg, "0x", 2 ) || !strncmp( optarg, "0X", 2 ) )
                rc = sscanf( optarg, "0x%x", &id_read );
            else
                rc = sscanf( optarg, "%u", &id_read );
            assert( rc == 1 );
            break;

        case 'w':
            if ( !strncmp( optarg, "0x", 2 ) || !strncmp( optarg, "0X", 2 ) )
                rc = sscanf( optarg, "0x%x", &id_write );
            else
                rc = sscanf( optarg, "%u", &id_write );
            assert( rc == 1 );
            break;

        default:
            cerr << "Error: getopt returned invalid character code '" << char(c) << "' = " << int(c) << ", giving up!\n";
            exit( 1 );
        }
    }

    if ( baudrate != 0 )
    {
        // a baudrate was specified on the command line, so overwrite the selected one
        if ( use_can )
            can_baudrate = baudrate;
        else
            rs232_baudrate = baudrate;
    }

    if ( do_print_version )
    {
        cout << "PC-side:\n";
        cout << "  Demo-program name:         " << argv[0] << "\n";
        cout << "  Demo-program revision:     " << version << "\n";
        cout << "  " << libname << " release:    " << librelease << "\n";

        try
        {
            g_sdh_debug_log = debuglog;
            cSDH hand( false, false, debug_level-1 );

            if ( use_can )
                hand.OpenCAN_ESD( net, can_baudrate, timeout, id_read, id_write );
            else
                hand.OpenRS232( port, rs232_baudrate, timeout );

            cout << "SDH-side:\n";
            cout << "  SDH firmware release:      " << hand.GetInfo( "release-firmware" ) << "\n";
            cout << "  SDH firmware date:         " << hand.GetInfo( "date-firmware" ) << "\n";
            cout << "  SDH SoC ID:                " << hand.GetInfo( "release-soc" ) << "\n";
            cout << "  SDH SoC date:              " << hand.GetInfo( "date-soc" ) << "\n";
            cout << "  SDH ID:                    " << hand.GetInfo( "id-sdh" ) << "\n";
            cout << "  SDH Serial Number:         " << hand.GetInfo( "sn-sdh" ) << "\n";
            hand.Close();
        }
        catch ( cSDHLibraryException* e )
        {
            cerr << "Could not get all version info from SDH: " << e->what() << "\n";
            delete e;
        }
        exit( 0 );
    }

    return optind;
    //
    //----------------------------------------------------------------------
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
