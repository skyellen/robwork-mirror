//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_demo_dsaoptions_cpp_general General file information

  \author   Dirk Osswald
  \date     2008-06-13

  \brief
  Implementation of a class to parse common SDH related command line options

  \section sdhlibrary_cpp_demo_dsaoptions_cpp_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_demo_dsaoptions_cpp_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
  \par SVN file revision:
  $Id: dsaoptions.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

  \subsection sdhlibrary_cpp_demo_dsaoptions_cpp_changelog Changelog of this file:
  \include dsaoptions.cpp.log

*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>

#include <iostream>
#include <fstream>

using namespace std;

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------

#include "sdh/dsa.h"
#include "sdh/sdhlibrary_settings.h"
#include "dsaoptions.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


static char* dsaoptions_usage =
    "options:\n"
    "  -h, --help            show this help message and exit\n"
    "  -p, --port=PORT, --dsaport=PORT \n"
    "                        use RS232 communication PORT to connect to to \n"
    "                        tactile sensor controller of SDH \n"
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
    "  -v, --version         Print the version (revision/release names of application,\n"
    "                        library and firmware then exit.\n"
    "  -r, --framerate=FRAMERATE \n"
    "                        report tactile sensor frames with FRAMERATE frames per second\n"
    "  -f, --fullframe       Print acquired full frames numerically.\n"
    "  -s, --sensorinfo      Print sensor info from DSA (texel dimensions, number of texels...).\n"
    "  -c, --controllerinfo  Print controller info from DSA (version...).\n"
    "  -m, --matrixinfo=MATRIX_INDEX\n"
    "                        Print matrix info for matrix with index MATRIX_INDEX from DSA.\n"
    "  --no_rle              Do not use the RunLengthEncoding\n"
    ;

static char* dsaoptions_short_options = "hp:r:fscmd:v";
static struct option dsaoptions_long_options[] =
    {
        // name, has_arg, flag, val
        {"help",           0, 0, 'h'},
        {"dsaport",        1, 0, 256+'p'},
        {"port",           1, 0, 'p'},
        {"framerate",      1, 0, 'r' },
        {"fullframe",      0, 0, 'f'},
        {"sensorinfo",     0, 0, 's'},
        {"controllerinfo", 0, 0, 'c' },
        {"matrixinfo",     1, 0, 'm'},
        {"no_rle",         0, 0, 256+'r' },
        {"debug",          1, 0, 'd'},
        {"debuglog",       1, 0, 'l'},
        {"version",        0, 0, 'v'},
        {0, 0, 0, 0}
    };


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------


cDSAOptions::cDSAOptions( void )
{
    usage = dsaoptions_usage;

    // set default options
    dsaport        = 0;    // 0=/dev/ttyS0=COM1
    debug_level    = 0;    // 0: no debug messages

    fullframe = false;
    framerate = -1;
    sensorinfo = false;
    controllerinfo = false;
    matrixinfo = -1;
    do_RLE = true;
    debuglog       = &cerr;
}

void cDSAOptions::Parse( int argc, char** argv,
                         char const* helptext, char const* progname, char const* version, char const* libname, char const* librelease )
{
    // parse options from command line
    bool do_print_version = false;
    int option_index = 0;
    int rc;

    while (1)
    {
        int c;
        c = getopt_long( argc, argv,
                         dsaoptions_short_options, dsaoptions_long_options,
                         &option_index );
        if (c == -1)
            break;

        switch (c)
        {
        case 'h':
            cout << helptext << "\n\nusage: " << progname << " [options]\n" << usage << "\n";
            exit(0);

        case 'p':
            // no break here
        case 256+'p':
            rc = sscanf( optarg, "%d", &dsaport );
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
        case 'v':
            do_print_version = true; // defer actual printing until all options are parsed (which might change the communication to use)
            break;

        case 'r':
            rc = sscanf( optarg, "%d", &framerate );
            assert( rc == 1 );
            break;
        case 'f':
            fullframe = true;
            break;
        case 's':
            sensorinfo = true;
            break;
        case 'c':
            controllerinfo = true;
            break;
        case 'm':
            rc = sscanf( optarg, "%d", &matrixinfo );
            assert( rc == 1 );
            break;
        case 256+'r':
            do_RLE = false;
            break;

        default:
            cerr << "Error: getopt returned invalid character code '" << char(c) << "' = " << int(c) << ", giving up!\n";
            exit( 1 );
        }
    }

    if ( optind < argc)
    {
        cerr << "Ignoring non-option ARGV-elements: ";
        while (optind < argc)
            cerr << "\"" << argv[optind++] << "\" ";
        cerr << "\n";
    }


    if ( do_print_version )
    {
        cout << "Demo-program revision:  " << version << "\n";
        cout << libname << " release: " << librelease << "\n";

        try
        {
            cDSA dsa( 0, dsaport );

            cout << "DSA controller info hw_version: " << (int) dsa.GetControllerInfo().hw_version << "\n";
            cout << "DSA controller info sw_version: " << (int) dsa.GetControllerInfo().sw_version << "\n";
            cout << "DSA sensor info hw_revision:    " << (int) dsa.GetSensorInfo().hw_revision << "\n";
            dsa.Close();
        }
        catch ( cDSAException* e )
        {
            cerr << "Could not get firmware release from DSA: " << e->what() << "\n";
            delete e;
        }
        exit( 0 );
    }
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
