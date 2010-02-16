//======================================================================
/*!
    \file
     \section sdhlibrary_cpp_demo_sdhoptions_h_general General file information

       \author   Dirk Osswald
       \date     2008-05-05

     \brief
       Implementation of a class to parse common %SDH related command line options

     \section sdhlibrary_cpp_demo_sdhoptions_h_copyright Copyright

     Copyright (c) 2008 SCHUNK GmbH & Co. KG

     <HR>
     \internal

       \subsection sdhlibrary_cpp_demo_sdhoptions_h_details SVN related, detailed file specific information:
         $LastChangedBy: Osswald2 $
         $LastChangedDate: 2009-11-02 09:20:24 +0100 (Mo, 02 Nov 2009) $
         \par SVN file revision:
           $Id: sdhoptions.h 4932 2009-11-02 08:20:24Z Osswald2 $

     \subsection sdhlibrary_cpp_demo_sdhoptions_h_changelog Changelog of this file:
         \include sdhoptions.h.log

*/
//======================================================================

#ifndef SDHOPTIONS_h
#define SDHOPTIONS_h

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <getopt.h>
#include <assert.h>

#include <iostream>
#include <string>

//----------------------------------------------------------------------
// Project Includes - include with ""
//---------------------------------------------------------------------


//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

//! string defining all the usage helptexts included by default
#define SDHUSAGE_DEFAULT "general sdhcom_serial sdhcom_common sdhcom_esdcan sdhcom_peakcan sdhcom_cancommon"

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


// class for option parsing holding option parsing results
class cSDHOptions
{
public:
    static int const MAX_DEV_LENGTH = 32;

    std::string   usage;

    int           debug_level;
    std::ostream* debuglog;

    int           sdhport;
    char          sdh_rs_device[MAX_DEV_LENGTH];
    double        timeout;
    unsigned long rs232_baudrate;

    bool          use_can_esd;
    int           net;

    bool          use_can_peak;
    char          sdh_canpeak_device[MAX_DEV_LENGTH];

    unsigned long can_baudrate;
    unsigned int  id_read;
    unsigned int  id_write;

    bool          use_radians;
    bool          use_fahrenheit;
    double        period;

    int           dsaport;
    char          dsa_rs_device[MAX_DEV_LENGTH];
    bool          do_RLE;

    int           framerate;
    bool          fullframe;
    bool          sensorinfo;
    bool          controllerinfo;
    int           matrixinfo;


    /*!
     * constructor: init members to their default values
     *
     * \param option_selection - string that names the options to
     *        include in helptext for online help. With a text including one of
     *        the following keywords the corresponding helptext is added
     *        to the usage helptext
     *        - "general" see sdhusage_general
     *        - "sdhcom_serial" see sdhusage_sdhcom_serial
     *        - "sdhcom_common" see sdhusage_sdhcom_common
     *        - "sdhcom_esdcan" see sdhusage_sdhcom_esdcan
     *        - "sdhcom_peakcan" see sdhusage_sdhcom_peakcan
     *        - "sdhcom_cancommon" see sdhusage_sdhcom_cancommon
     *        - "sdhother" see sdhusage_sdhother
     *        - "dsacom" see sdhusage_dsacom
     *        - "dsaother" see sdhusage_dsaother
     */
    cSDHOptions( char const* option_selection = SDHUSAGE_DEFAULT );

    /*! parse the command line parameters \a argc, \a argv into members. \a helptext, \a progname, \a version, \a libname and \a librelease are used when printing online help.
        start parsing at option with index *p_option_index
        parse all options if parse_all is true, else only one option is parsed

        \return the optind index of the first non option argument in argv
     */
    int Parse( int argc, char** argv,
               char const* helptext, char const* progname, char const* version, char const* libname, char const* librelease );
};


#endif

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
