//======================================================================
/*!
    \file

       \author   Dirk Osswald
       \date     2008-05-05

     \brief
       Implementation of a class to parse common %SDH related command line options

     \section sdhlibrary_cpp_demo_sdhoptions_cpp_copyright Copyright

     Copyright (c) 2008 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_demo_sdhoptions_cpp_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2009-12-01 11:41:18 +0100 (Di, 01 Dez 2009) $
         \par SVN file revision:
           $Id: sdhoptions.cpp 5000 2009-12-01 10:41:18Z Osswald2 $

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
#include "sdh/release.h"
#include "sdh/dsa.h"
#include "sdhoptions.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//! general options
static char const* sdhusage_general =
    //2345678901234567890123456789012345678901234567890123456789012345678901234567890
    //        1         2         3         4         5         6         7         8
    "General options:\n"
    "  -h, --help\n"
    "      Show this help message and exit.\n"
    "      \n"
    "  -v, --version\n"
    "      Print the version (revision/release names) and dates of application,\n"
    "      library (and the attached SDH firmware, if found), then exit.\n"
    "      \n"
    "  -d LEVEL, --debug=LEVEL\n"
    "      Print debug messages of level LEVEL or lower while executing the program.\n"
    "      Level 0 (default): No messages,  1: application-level messages, \n"
    "      2: cSDH-level messages, 3: cSDHSerial-level messages\n"
    "      \n"
    "  -l LOGFILE, --debuglog=LOGFILE\n"
    "      Redirect the printed debug messages to LOGFILE instead of default \n"
    "      standard error. If LOGFILE starts with '+' then the output will be \n"
    "      appended to the file (without the leading '+'), else the file will be\n"
    "      overwritten.\n"
    "      \n"
    ;

//! RS232 communication options
static char const* sdhusage_sdhcom_serial =
    "Communication options:\n"
    "  -p PORT, --port=PORT, --sdhport=PORT\n"
    "      Use RS232 communication PORT to connect to the SDH instead of the default\n"
    "      0='COM1'='/dev/ttyS0'.\n"
    "      \n"
#if ! SDH_USE_VCC
    "  --sdh_rs_device=DEVICE_FORMAT_STRING\n"
    "      Use DEVICE_FORMAT_STRING instead of the default \"/dev/ttyS%d\". Useful\n"
    "      e.g. to use USB to RS232 converters available via \"/dev/ttyUSB%d\". \n"
    "      If the DEVICE_FORMAT_STRING contains '%d' then the PORT must also be \n"
    "      provided. If not then the DEVICE_FORMAT_STRING is the full device name. \n"
    "      \n"
#endif
    ;

//! Common communication options
static char const* sdhusage_sdhcom_common =
    "  -T TIMEOUT, --timeout=TIMEOUT Timeout in seconds when waiting for data from\n"
    "      SDH. The default -1 means: wait forever.\n"
    "      \n"
    "  -b BAUDRATE, --baud=BAUDRATE\n"
    "      Use BAUDRATE in bit/s for communication. Default=115200 Bit/s for RS232\n"
#if WITH_ESD_CAN || WITH_PEAK_CAN
    "      and 1000000 Bit/s (1MBit/s) for CAN\n"
#endif
    ;

//! ESD CAN communication options
static char const* sdhusage_sdhcom_esdcan =
#if WITH_ESD_CAN
    "  -c, --can, --canesd\n"
    "      Use CAN bus via an ESD adapter to connect to the SDH instead of RS232.\n"
    "      \n"
    "  -n NET, --net=NET\n"
    "      Use ESD CAN NET for CAN communication, default=0.\n"
    "      \n"
#else
    ""
#endif
    ;
//! PEAK CAN communication options
static char const* sdhusage_sdhcom_peakcan =
#if WITH_PEAK_CAN
    "  --canpeak\n"
    "      Use CAN bus via a PEAK adapter to connect to the SDH instead of RS232.\n"
    "      \n"
#else
    ""
#endif
#if WITH_PEAK_CAN &&  defined( OSNAME_LINUX )
    "  --sdh_canpeak_device=DEVICE_NAME\n"
    "      Use DEVICE_NAME instead of the default \"/dev/pcanusb0\"."
    "      \n"
#endif
    ;
//! Common CAN communication options
static char const* sdhusage_sdhcom_cancommon =
#if WITH_ESD_CAN || WITH_PEAK_CAN
    "  -e ID_READ, --id_read=ID_READ\n"
    "      Use CAN ID ID_READ for receiving CAN messages (default: 43=0x2B).\n"
    "      \n"
    "  -w ID_WRITE, --id_write=ID_WRITE\n"
    "      Use CAN ID ID_WRITE for writing CAN messages (default: 42=0x2A).\n"
    "      \n"
#else
    ""
#endif
    ;
//! Other options
static char const* sdhusage_sdhother =
    "Other options:\n"
    "  -R, --radians\n"
    "      Use radians and radians per second for angles and angular velocities\n"
    "      instead of default degrees and degrees per second.\n"
    "      \n"
    "  -F, --fahrenheit\n"
    "      Use degrees fahrenheit to report temperatures instead of default degrees\n"
    "      celsius.\n"
    "      \n"
    "  -t PERIOD, --period=PERIOD\n"
    "      For periodic commands only: Time period of measurements in seconds. The\n"
    "      default of '0' means: report once only. If set then the time since start\n"
    "      of measurement is printed at the beginning of every line.\n"
    "      \n"
    ;
//! DSA (tactile sensor) communication options
static char const* sdhusage_dsacom =
    "DSA options (tactile sensor):\n"
    "  -q PORT, --dsaport=PORT\n"
    "      use RS232 communication PORT to connect to to  tactile sensor controller\n"
    "      of SDH  instead of default 1='COM2'='/dev/ttyS1'.\n"
    "      \n"
#if ! SDH_USE_VCC
    "  --dsa_rs_device=DEVICE_FORMAT_STRING\n"
    "      Use DEVICE_FORMAT_STRING instead of the default \"/dev/ttyS%d\". Useful\n"
    "      e.g. to use USB to RS232 converters available via \"/dev/ttyUSB%d\".  If \n"
    "      the DEVICE_FORMAT_STRING contains '%d' then the dsa PORT must also be \n"
    "      provided. If not then the DEVICE_FORMAT_STRING is the full device name.\n"
    "      \n"
#endif
    "  --no_rle\n"
    "      Do not use the RunLengthEncoding\n"
    "      \n"
    ;
//! DSA (tactile sensor) other options
static char const* sdhusage_dsaother =
    "  -r, --framerate=FRAMERATE\n"
    "      Framerate for acquiring full tactile sensor frames.  Default value 0\n"
    "      means 'acquire a single frame only'.  Any value > 0 will make the\n"
    "      DSACON32m controller in the SDH send data at the highest possible rate \n"
    "      (ca. 30 FPS (frames per second)).\n"
    "      \n"
    "  -f, --fullframe\n"
    "      Print acquired full frames numerically.\n"
    "      \n"
    "  -S, --sensorinfo\n"
    "      Print sensor info from DSA (texel dimensions, number of texels...).\n"
    "      \n"
    "  -C, --controllerinfo\n"
    "      Print controller info from DSA (version...).\n"
    "      \n"
    "  -M, --matrixinfo=MATRIX_INDEX\n"
    "      Print matrix info for matrix with index MATRIX_INDEX from DSA.\n"
    "      \n"
    ;

static char const* sdhoptions_short_options = "hvd:l:p:T:b:cn:e:w:RFt:q:r:fSCM:";
static struct option sdhoptions_long_options[] =
{
  // name              , has_arg, flag, val
  {"help"              , 0      , 0   , 'h'      },
  {"version"           , 0      , 0   , 'v'      },
  {"debug"             , 1      , 0   , 'd'      },
  {"debuglog"          , 1      , 0   , 'l'      },

  {"port"              , 1      , 0   , 'p'      },
  {"sdhport"           , 1      , 0   , 'p'      },
  {"sdh_rs_device"     , 1      , 0   , 'S' + 256},
  {"timeout"           , 1      , 0   , 'T'      },
  {"baud"              , 1      , 0   , 'b'      },

  {"can"               , 0      , 0   , 'c'      },
  {"canesd"            , 0      , 0   , 'c'      },
  {"net"               , 1      , 0   , 'n'      },

  {"canpeak"           , 0      , 0   , 'p' + 256},
  {"sdh_canpeak_device", 1      , 0   , 'P' + 256},

  {"id_read"           , 1      , 0   , 'e'      },
  {"id_write"          , 1      , 0   , 'w'      },

  {"radians"           , 0      , 0   , 'R'      },
  {"fahrenheit"        , 0      , 0   , 'F'      },
  {"period"            , 1      , 0   , 't'      },

  {"dsaport"           , 1      , 0   , 'q'      },
  {"dsa_rs_device"     , 1      , 0   , 'D' + 256},
  {"no_rle"            , 0      , 0   , 'r' + 256},

  {"framerate"         , 1      , 0   , 'r'      },
  {"fullframe"         , 0      , 0   , 'f'      },
  {"sensorinfo"        , 0      , 0   , 'S'      },
  {"controllerinfo"    , 0      , 0   , 'C'      },
  {"matrixinfo"        , 1      , 0   , 'M'      },

  {0, 0, 0, 0}
};


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


cSDHOptions::cSDHOptions( char const* option_selection )
{
    std::string os( option_selection );

    if ( os.find( "general" ) !=  string::npos )
        usage.append( sdhusage_general );
    if ( os.find( "sdhcom_serial" ) != string::npos )
        usage.append( sdhusage_sdhcom_serial );
    if ( os.find( "sdhcom_common" ) != string::npos )
        usage.append( sdhusage_sdhcom_common );
    if ( os.find( "sdhcom_esdcan" ) != string::npos )
        usage.append( sdhusage_sdhcom_esdcan );
    if ( os.find( "sdhcom_peakcan" ) != string::npos )
        usage.append( sdhusage_sdhcom_peakcan );
    if ( os.find( "sdhcom_cancommon" ) != string::npos )
        usage.append( sdhusage_sdhcom_cancommon );
    if ( os.find( "sdhother" ) != string::npos )
        usage.append( sdhusage_sdhother );
    if ( os.find( "dsacom" ) != string::npos )
        usage.append( sdhusage_dsacom );
    if ( os.find( "dsaother" ) != string::npos )
        usage.append( sdhusage_dsaother );

    // set default options
    debug_level        = 0;    // 0: no debug messages
    debuglog           = &cerr;

    sdhport            = 0;    // 0=/dev/ttyS0=COM1
    strncpy( sdh_rs_device, "/dev/ttyS%d", MAX_DEV_LENGTH );
    timeout            = -1.0; // no timeout, wait forever (which is what we need)
    rs232_baudrate     = 115200;

    use_can_esd        = false;
    net                = 0;

    use_can_peak       = false;
    strncpy( sdh_canpeak_device, "/dev/pcanusb0", MAX_DEV_LENGTH );

    can_baudrate       = 1000000;
    id_read            = 43;
    id_write           = 42;

    use_radians        = false;
    use_fahrenheit     = false;
    period             = 0.0;  // no period, read only once

    dsaport            = 0;    // 0=/dev/ttyS0=COM1
    strncpy( dsa_rs_device, "/dev/ttyS%d", MAX_DEV_LENGTH );
    do_RLE             = true;

    framerate          = -1;
    fullframe          = false;
    sensorinfo         = false;
    controllerinfo     = false;
    matrixinfo         = -1;

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

        case 'v':
            do_print_version = true; // defer actual printing until all options are parsed (which might change the communication to use)
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

        case 'p':
            rc = sscanf( optarg, "%d", &sdhport );
            assert( rc == 1 );
            break;

        case 'S'+256:
            strncpy( sdh_rs_device, optarg, MAX_DEV_LENGTH );
            break;

        case 'T':
            rc = sscanf( optarg, "%lf", &timeout );
            assert( rc == 1 );
            break;

        case 'b':
            rc = sscanf( optarg, "%lu", &baudrate ); // store in intermediate variable since -b might be specified before --can
            assert( rc == 1 );
            break;

        //---
        case 'c':
            use_can_esd = true;
            break;

        case 'n':
            rc = sscanf( optarg, "%d", &net );
            assert( rc == 1 );
            break;

        //---
        case 'p'+256:
            use_can_peak = true;
            break;

        case 'P'+256:
            strncpy( sdh_canpeak_device, optarg, MAX_DEV_LENGTH );
            break;


        //---
        case 'e':
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

        //---
        case 'R':
            use_radians = true;
            break;

        case 'F':
            use_fahrenheit = true;
            break;

        case 't':
            rc = sscanf( optarg, "%lf", &period );
            assert( rc == 1 );
            break;

        //---
        case 'q':
            rc = sscanf( optarg, "%d", &dsaport );
            assert( rc == 1 );
            break;

        case 'D'+256:
            strncpy( dsa_rs_device, optarg, MAX_DEV_LENGTH );
            break;

        case 256+'r':
            do_RLE = false;
            break;

        //---
        case 'r':
            rc = sscanf( optarg, "%d", &framerate );
            assert( rc == 1 );
            break;
        case 'f':
            fullframe = true;
            break;
        case 'S':
            sensorinfo = true;
            break;
        case 'C':
            controllerinfo = true;
            break;
        case 'M':
            rc = sscanf( optarg, "%d", &matrixinfo );
            assert( rc == 1 );
            break;

        //---

        default:
            cerr << "Error: getopt returned invalid character code '" << char(c) << "' = " << int(c) << ", giving up!\n";
            exit( 1 );
        }
    }

    if ( baudrate != 0 )
    {
        // a baudrate was specified on the command line, so overwrite the selected one
        if ( use_can_esd || use_can_peak )
            can_baudrate = baudrate;
        else
            rs232_baudrate = baudrate;
    }

    char const* libdate = PROJECT_DATE;

    if ( do_print_version )
    {
        cout << "PC-side:\n";
        cout << "  Demo-program name:         " << argv[0] << "\n";
        cout << "  Demo-program revision:     " << version << "\n";
        cout << "  " << libname << " release:    " << librelease << "\n";
        cout << "  " << libname << " date:       " << libdate << "\n";
        cout.flush();

        try
        {
            g_sdh_debug_log = debuglog;
            cSDH hand( false, false, debug_level-1 );

            if      ( use_can_esd )
                hand.OpenCAN_ESD( net, can_baudrate, timeout, id_read, id_write );
            else if ( use_can_peak )
                hand.OpenCAN_PEAK( can_baudrate, timeout, id_read, id_write, sdh_canpeak_device );
            else
                hand.OpenRS232( sdhport, rs232_baudrate, timeout, sdh_rs_device );

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

        try
        {
            cDSA dsa( 0, dsaport );

            cout << "DSA-side:\n";
            cout << "  DSA controller info hw_version: " << (int) dsa.GetControllerInfo().hw_version << "\n";
            cout << "  DSA controller info sw_version: " << (int) dsa.GetControllerInfo().sw_version << "\n";
            cout << "  DSA sensor info hw_revision:    " << (int) dsa.GetSensorInfo().hw_revision << "\n";
            dsa.Close();
        }
        catch ( cDSAException* e )
        {
            cerr << "Could not get sensor controller firmware release from DSA: " << e->what() << "\n";
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
