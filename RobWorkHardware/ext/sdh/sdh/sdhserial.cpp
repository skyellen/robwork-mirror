//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_sdhserial_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Interface of class #SDH::cSDHSerial.

  \section sdhlibrary_cpp_sdhserial_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_sdhserial_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-13 17:07:24 +0200 (Mo, 13 Okt 2008) $
      \par SVN file revision:
        $Id: sdhserial.cpp 3686 2008-10-13 15:07:24Z Osswald2 $

  \subsection sdhlibrary_cpp_sdhserial_cpp_changelog Changelog of this file:
      \include sdhserial.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "util.h"
#include "sdhserial.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------

cSDHSerial::cSDHSerial( int _debug_level )
    :
    // call base class constructors:
    cSDHBase( _debug_level ),

    // init member objects:
    com( NULL )
{
    //---------------------
    // Option handling:

    // use green as color for messages from cSDHSerial
    cdbg.SetColor( "green" );
    cdbg << "Debug messages of cSDHSerial are printed like this.\n";

    //---------------------
    // initialize additional member variables:

    m_sequtime = 0.0; // !!!

    //! String to use as "End Of Line" marker when sending to SDH

    EOL="\r\n";
    //---------------------
}
//-----------------------------------------------------------------


void cSDHSerial::Open( cSerialBase* _com )
    throw (cSDHLibraryException*)
{
    com = _com;

    if (com == NULL)
    {
        // "virtual" communication for offline tests
        cdbg << "!!! Opened virtual communication for offline testing!!!\n";
        return;
    }

    assert( com != NULL );

    //---------------------
    // open connection to SDH:

    nb_lines_to_ignore = 0;

    com->Open();
    // the above call will succeed even if the hand is connected but off
    //---------------------


    //---------------------
    // Clean up communication line. (To make shure that no previous
    // communication that was received only partly by the SDH will
    // lead to an error when sending the "ver" command below. This
    // can happen if a PC program gets interrupted while in the
    // middle of sending a command to the SDH. Then any new
    // command for the SDH sent by the next PC program will
    // confuse the SDH

    double old_timeout = com->GetTimeout();
    com->SetTimeout( 1.0 );
    try
    {
        Send( " " ); // empty command to terminate any potential partly received previous command
    }
    catch (cSDHErrorCommunication* e)
    {
        cdbg << "caught <" << *e << "> (ignored while cleaning up communication)\n";
    }
    //---------------------


    //---------------------
    // to make shure that the SDH is connected:
    // try to get the firmware version with timeout
    try
    {
        Send( "ver" );
    }
    catch (cSDHErrorCommunication* e)
    {
        cdbg << "caught <" << *e << ">\n";
        //std::cerr << "SDHLibrary-CPP: cSDHSerial.Open(): Timeout while trying to get SDH firmware version\n  Is the SDH really connected and powered?\n";

        Close(); // make shure this cSDHSerial object does not appear open in case of error
        com->SetTimeout( old_timeout );

        cSDHErrorCommunication*  e2 = new cSDHErrorCommunication( cMsg( "Could not connect to SDH firmware! Is the SDH really connected and powered? (Sending \"ver\" caused: %s)", e->what() ) );
        delete e;

        throw e2;
    }

    com->SetTimeout( old_timeout );
    //---------------------
}
//-----------------------------------------------------------------


void cSDHSerial::Close()
    throw (cSDHLibraryException*)
{
    if ( com == NULL )
    {
        // "virtual" communication for offline tests
        cdbg << "!!! Closed virtual communication for offline testing!!!\n";
        return;
    }

    com->Close();
}
//-----------------------------------------------------------------


bool cSDHSerial::IsOpen( void )
{
    return com != NULL  &&  com->IsOpen();
}
//----------------------------------------------------------------------



void cSDHSerial::Send( char const* s, int nb_lines, int nb_lines_total, int max_retries )
//void cSDHSerial::Send( char const* s, int nb_lines, int nb_lines_total )
    throw( cSDHLibraryException* )
{
    if (com==NULL)
    {
        // "virtual" communication for offline tests
        cdbg << "!!! Virtual communication, ignoring sending '" << s << "'\n";
        return;
    }

    int retries = max_retries; // retry sending at most this many times
    while (retries > 0)
    {
        try
        {
            //---------------------
            // first read all lines to ignore (replies of previous commands)
            while ( nb_lines_to_ignore > 0 )
            {
                com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
                nb_lines_to_ignore -= 1;
                cdbg << "ignoring line <" << reply.CurrentLine() << ">\n";

                reply.Reset();
            }
            //---------------------


            firmware_state = eEC_SUCCESS;
            reply.Reset();

            //---------------------
            // send new command to SDH
            cdbg << "cSDHSerial::Send: sending command '" << s << "' to SDH\n";
            cdbg << "  nb_lines=" << nb_lines << "  nb_lines_total=" << nb_lines_total << "  nb_lines_to_ignore=" << nb_lines_to_ignore << "\n";

            com->write( s );
            com->write( EOL );

            cdbg << "sent command\n";
            //---------------------

            //---------------------
            // read reply if requested
            while (nb_lines == All || nb_lines > 0)
            {

                //---------------------
                // now read requested reply lines of current command
                com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
                cdbg << "read line '" << reply.CurrentLine() << "'\n";
                if (nb_lines != All)
                {
                    nb_lines -= 1;
                }
                if (nb_lines_total != All)
                {
                    nb_lines_total -= 1;
                }

                // remove beginning and trailing "\r\n" of the line just read

                //   remove beginning "\r\n"-s:
                char* startp = reply.CurrentLine();

                startp[ reply.eMAX_CHARS ] = '\0';
                while ( strchr( "\r\n", *startp ) != NULL )
                {
                    startp += 1;
                }
                if ( startp != reply.CurrentLine() )
                {
                    memmove( reply.CurrentLine(), startp, strlen( startp ) );
                }

                //   remove trailing "\r\n"-s:
                char* endp = reply.CurrentLine() + strlen( reply.CurrentLine() )-1;
                while ( strchr( "\r\n", *endp ) != NULL )
                {
                    *endp = '\0';
                    endp -= 1;
                }

                cdbg << "appending cleaned up line '" << reply.CurrentLine() << "'\n";
                if (reply.CurrentLine()[0] != '@')
                {
                    break;
                }
                //---------------------
            }

            //---------------------
            // remember if there are more lines to be ignored next time
            if (nb_lines_total != All)
            {
                nb_lines_to_ignore = nb_lines_total;
            }
            cdbg << nb_lines_to_ignore <<" lines remain to be ignored\n";
            //---------------------

            //---------------------
            // set state if possible
            if (nb_lines_to_ignore == 0)
            {
                ExtractFirmwareState();
            }
            //---------------------

            // finished, so no more retries needed
            retries = 0;

        } // end of try
        catch (cSDHErrorCommunication* e)
        {
            // some communication error occured, so retry:
            retries -= 1;
            if (retries <= 0)
            {
                cdbg << "Retried sending, but still got errors from SDH!\n";
                // rethrow e:
                throw;
            }
            cdbg << "ignoring cSDHErrorCommunication: " << *e << "\n";

            // resync first:
            Sync();
            //now start over again

        } // end of catch
    } // end of while (retries > 0)
    cdbg << "got reply: " << reply;
}
//-----------------------------------------------------------------


void cSDHSerial::ExtractFirmwareState()
    throw (cSDHErrorCommunication*)
{
    //---------------------
    // check first char of last line of lines

    if   (reply[-1][0] == 'E')
    {
        // it is an error message:
        sscanf( reply[-1] +1, "%d", (int*) (&firmware_state) );
        cdbg << "got error reply '" << reply[-1] << "' = " << firmware_state << " = " << firmware_error_codes[firmware_state] << "\n";
        throw new cSDHErrorCommunication( cMsg( "SDH firmware reports error %d = %s", firmware_state, firmware_error_codes[firmware_state]) );
    }

    else if (reply[-1][0] == '@')
    {
        // it is an debug message (should not happen):
        throw new cSDHErrorCommunication( cMsg( "Cannot get firmware state from lines" ) );
    }

    else
    {
        // it is a normal "command completed" line:
        firmware_state = eEC_SUCCESS;
    }
}
//-----------------------------------------------------------------


double cSDHSerial::GetDuration( char* line )
    throw (cSDHErrorCommunication*)
{
    char* pattern_at = strstr( line, "T=" );

    if (pattern_at == NULL)
        throw new cSDHErrorCommunication( cMsg( "Could not extract duration from lines '%s'", line ) );

    double duration;
    sscanf( pattern_at, "T=%lf", &duration );

    cdbg << "extracted duration " << duration << "\n";
    return duration;
}
//-----------------------------------------------------------------


double cSDHSerial::get_duration( void )
{
    // Actual input/output for the command looks like:
    //--
    // get_duration
    // @max distance=45.06, T=4.51s, num_points: 451
    // GET_DURATION=4.51

    //---------------------
    // settings for sequ/non-sequ:
    int nb_lines_total = 2;
    int nb_lines = nb_lines_total;
    //---------------------

    //---------------------
    // send command and parse reply
    Send( "get_duration", nb_lines, nb_lines_total );
    double T = GetDuration( reply[nb_lines_total-2] );
    //---------------------

    return T;
}
//----------------------------------------------------------------------


void cSDHSerial::Sync( )
    throw( cSDHErrorCommunication* )
{
    // first read all lines to ignore (replies of previous commands)
    while ( nb_lines_to_ignore > 0 )
    {
        com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n" );
        nb_lines_to_ignore -= 1;
        cdbg << "syncing: ignoring line <" << reply.CurrentLine() << ">\n";

        reply.Reset();
    }
    if (reply.Length() > 0)
        ExtractFirmwareState();
}
//-----------------------------------------------------------------

void cSDHSerial::SyncUnknown( )
    throw( cSDHErrorCommunication* )
{
    // read all lines until timeout
    while (1)
    {
        try
        {
            com->readline( reply.NextLine(), reply.eMAX_CHARS, "\n", true );
            cdbg << "syncing unknown: ignoring line <" << reply.CurrentLine() << ">\n";

            reply.Reset();
        }
        catch (cSerialBaseException* e)
        {
            cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSerialBaseException: " << *e << "\n";
            break;
        }
    }
    if (reply.Length() > 0)
        ExtractFirmwareState();
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::AxisCommand( char* command, int axis, double* value )
    throw (cSDHLibraryException*)
{
#if SDH_USE_VCC
    int cutoff  = static_cast<int>(strlen(command)) + 1;
#else
    int cutoff  = strlen( command ) + 1;
#endif
    int cutoff1 = cutoff +3;
    char cmd[ cSimpleStringList::eMAX_CHARS ];
    int retries = 3; // retry sending at most this many times
    while (retries > 0)
    {
        try
        {
            if (axis == All && value == NULL)
            {
                Send( command );
                return cSimpleVector( NUMBER_OF_AXES, reply[0]+cutoff );
            }

            if (axis != All)
            {
                CheckIndex( axis, NUMBER_OF_AXES, "axis" );
                if (value == NULL)
                {
#if SDH_USE_VCC
      //_snprintf_s( cmd, cSimpleStringList::eMAX_CHARS, cSimpleStringList::eMAX_CHARS-1, "%s(%d)", command, axis );
      snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)", command, axis );//MINGW32
#else
      snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)", command, axis );
#endif
                    Send( cmd );
                    return cSimpleVector( 1, axis, reply[0]+cutoff1 );
                }

#if SDH_USE_VCC
      //_snprintf_s( cmd, cSimpleStringList::eMAX_CHARS, cSimpleStringList::eMAX_CHARS-1, "%s(%d)=%12.3f", command, axis, *value );
      snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)=%12.3f", command, axis, *value ); //MINGW32
#else
                snprintf( cmd, cSimpleStringList::eMAX_CHARS-1, "%s(%d)=%12.3f", command, axis, *value );
#endif
                Send( cmd );
                return cSimpleVector( 1, axis, reply[0]+cutoff1 );
            }

            if (axis == All)
            {
                Send( cMsg( "%s=%f,%f,%f,%f,%f,%f,%f", command, value[0], value[1], value[2], value[3], value[4], value[5], value[6] ).c_str() );
                return cSimpleVector( NUMBER_OF_AXES, reply[0] + cutoff );
            }

            throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in AxisCommand( command=%s, axis=%d, value=%p )'", command, axis, value ) );

        } // end of try
        //catch (cSimpleVectorException* e)
        catch (cSDHLibraryException* e)
        {
            // these errors seem to happen on linux only (not cygwin) where a reply can be partly received:

            // assume some communication error occured, so retry:
            retries -= 1;
            if (retries > 0)
                //cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSimpleVectorException: " << *e << "\n";
                cdbg << __FILE__ << ":" << __LINE__ << " ignoring cSDHLibraryException: " << *e << " (retrying)\n";
            else
            {
                cdbg << __FILE__ << ":" << __LINE__ << " retried but giving up now\n";
                // rethrow e:
                throw;
            }

            // resync first:
            Sync();
            //now start over again

        } // end of catch

    } // end of while (retries > 0)

    cdbg << "Retried sending, but still didnt work!\n";
    throw new cSDHLibraryException( "cSDHLibraryException", cMsg( "Unknown error while retrying" ) );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::pid( int axis, double* p, double* i, double* d )
    throw (cSDHLibraryException*)
{
    CheckIndex( axis, NUMBER_OF_AXES, "axis" );

    if (p == NULL  &&  i == NULL  &&  d == NULL)
    {
        Send( cMsg( "pid(%d)", axis ).c_str() );
        return cSimpleVector( 3, reply[0] + 7 );
    }
    if (p != NULL  &&  i != NULL  &&  d != NULL)
    {
        Send( cMsg( "pid(%d)=%f,%f,%f", axis, *p, *i, *d ).c_str() );
        return cSimpleVector( 3, reply[0]+7 );
    }

    throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in call' pid(axis=%d, p=%f, i=%f, d=%f )'", axis, *p, *i, *d ) );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::kv( int axis, double* kv )
    throw (cSDHLibraryException*)
{
    if (axis == All)
    {
        // SDH firmware cannot handle setting / getting all values at once
        // so emulate that

        // get/set all kv:

        cSimpleVector rv;

        for ( int i=0; i < NUMBER_OF_AXES; i++ )
        {
        cSimpleVector rvi;

        if (kv == NULL)
            rvi = AxisCommand( "kv", i, NULL );
        else
            rvi = AxisCommand( "kv", i, &(kv[i]) );
        rv[i] = rvi[i];
        }
        return rv;
    }
    else
    {
        return AxisCommand( "kv", axis, kv );
    }
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::ilim( int axis, double* limit )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "ilim", axis, limit );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::power( int axis, double* flag )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // power=0,0,0,0,0,0,0
    // POWER=0,0,0,0,0,0,0

    return AxisCommand( "power", axis, flag );
}
//----------------------------------------------------------------------


void cSDHSerial::demo( bool onoff )
{
    Send( cMsg( "demo=%d", int( onoff ) ).c_str() );
}
//-----------------------------------------------------------------


int cSDHSerial::property( char* propname, int value )
{
    Send( cMsg( "%s=%d", propname, value ).c_str() );
    int v;
    sscanf( reply[0] + strlen(propname), "%d", &v );
    return v;
}
//-----------------------------------------------------------------


int cSDHSerial::user_errors( int value )
{
    return property( "user_errors", value );
}
//-----------------------------------------------------------------


int cSDHSerial::terminal( int value )
{
    return property( "terminal", value );
}
//-----------------------------------------------------------------


int cSDHSerial::debug( int value )
{
    return property( "debug", value );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::v( int axis, double* velocity )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "v", axis, velocity );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::vlim( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "vlim", axis, NULL );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::a( int axis, double* acceleration )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "a", axis, acceleration );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::p( int axis, double* angle )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "p", axis, angle );
}
//-----------------------------------------------------------------


double cSDHSerial::m( bool sequ )
    throw(cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // m
    // @Enabling all axis
    // @max distance=45.06, T=4.51s, num_points: 451
    // m

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 3; ////10; //// !!!
    int nb_lines = nb_lines_total; // firmware bug: is always non-sequential in firmware!
    //---------------------

    //---------------------
    // send command and parse reply
    Send( "m", nb_lines, nb_lines_total );


    double T =  GetDuration( reply[1] );
    //---------------------

    // the SDH firmware does NOT produce an output after the command has finished
    if (sequ)
        // so just wait as long as the movement will take
        SleepSec( T + m_sequtime );

    return T;
}
//-----------------------------------------------------------------


void cSDHSerial::stop( void )
    throw (cSDHLibraryException*)
{
    Send( "stop" );
}
//----------------------------------------------------------------------


cSDHBase::eVelocityProfile cSDHSerial::vp( eVelocityProfile velocity_profile )
    throw (cSDHLibraryException*)
{
    char cmd[5];
    if ( velocity_profile < 0 )
        sprintf( cmd, "vp" );
    else if ( velocity_profile < eVP_DIMENSION )
        sprintf( cmd, "vp=%d", velocity_profile );
    else
        throw new cSDHErrorInvalidParameter( cMsg( "Invalid parameter in vp( velocity_profile=%d )'", velocity_profile ) );

    Send( cmd );

    int new_vp;
    sscanf( reply[0]+3, "%d", &new_vp );
    return (eVelocityProfile) new_vp;
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::pos( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "pos", axis );
}
//----------------------------------------------------------------------

cSimpleVector cSDHSerial::pos_save( int axis, double* value )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "pos_save", axis, value );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::ref( int axis, double* value )
    throw (cSDHLibraryException*)
{
    // !!! return AxisCommand( "ref", axis, value ); does not work since reply is verbose and delayed

    char cmd[32];

    sprintf( cmd, "ref(%d)=%d", axis, (int) *value );

    Send( cmd, All, All, 1 );

    // we have to ignore replies since number of reply lines depends on actual length of movement

    // !!! (This is really a mess but we cannot get around it since the SDH firmware sends kinky data and we cannot change the firmware of SDH 003)
    SyncUnknown();
    //cdbg << "sleeping 3s\n";
    SleepSec(3);
    //cdbg << "ok\n";
    SyncUnknown();
    SleepSec(1);
    SyncUnknown();


    return cSimpleVector( 1, axis, cmd+7 );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::vel( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "vel", axis );
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::state( int axis, double* dummy )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "state", axis );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::temp( void )
    throw(cSDHLibraryException*)
{
    cSimpleVector rv;

    Send( "temp" );

    sscanf( reply[0] + 5, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &(rv[0]), &(rv[1]), &(rv[2]), &(rv[3]), &(rv[4]), &(rv[5]), &(rv[6]) );
    return rv;
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::temp_electronics( void )
    throw(cSDHLibraryException*)
{
    cSimpleVector dummy;
    cSimpleVector rv;

    Send( "temp" );

    sscanf( reply[0] + 5, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
            &(dummy[0]), &(dummy[1]), &(dummy[2]), &(dummy[3]), &(dummy[4]), &(dummy[5]), &(dummy[6]),
            &(rv[0]), &(rv[1]) );
    return rv;
}
//-----------------------------------------------------------------


char* cSDHSerial::ver( void  )
    throw(cSDHLibraryException*)
{
    Send( "ver" );
    return reply[0] + 4;
}
//-----------------------------------------------------------------


char* cSDHSerial::ver_date( void  )
    throw(cSDHLibraryException*)
{
    Send( "ver_date" );
    return reply[0] + 9;
}
//-----------------------------------------------------------------


char* cSDHSerial::id( void )
    throw(cSDHLibraryException*)
{
    Send( "id" );
    return reply[0] + 3;
}
//-----------------------------------------------------------------


char* cSDHSerial::sn( void )
    throw(cSDHLibraryException*)
{
    Send( "sn" );
    return reply[0] + 3;
}
//-----------------------------------------------------------------


char* cSDHSerial::soc( void )
    throw(cSDHLibraryException*)
{
    Send( "soc" );
    return reply[0] + 4;
}
//-----------------------------------------------------------------


char* cSDHSerial::soc_date( void )
    throw(cSDHLibraryException*)
{
    Send( "soc_date" );
    return reply[0] + 9;
}
//-----------------------------------------------------------------


int cSDHSerial::numaxis( void )
    throw(cSDHLibraryException*)
{
    Send( "numaxis" );
    ////!!!return int( reply[0]+8 ); // did this ever work???
    int numaxis;
    sscanf( reply[0]+8, "%d", &numaxis );
    return numaxis;
}
//----------------------------------------------------------------------


cSimpleVector cSDHSerial::igrip( int axis, double* limit )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "igrip", axis, limit );
}
//-----------------------------------------------------------------


cSimpleVector cSDHSerial::ihold( int axis, double* limit )
    throw (cSDHLibraryException*)
{
    return AxisCommand( "ihold", axis, limit );
}
//-----------------------------------------------------------------


double cSDHSerial::selgrip( eGraspId grip, bool sequ )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // selgrip=1
    // @Krit prox angle -0.00
    // @Joint finger 1 proximal (-34.72 deg) is not critical.
    // @Joint finger 2 proximal (-34.54 deg) is not critical.
    // @Joint finger 3 proximal (-35.15 deg) is not critical.
    // @Enabling all axis
    // @Setting current limit to @1.0 @0.5 @0.5 @0.5 @0.5 @0.5 @0.5 @
    // @max distance=0.00, T=0.00s, num_points: 1
    // @max distance=0.00, T=0.00s, num_points: 1
    // @Setting current limit to @0.1 @0.2 @0.2 @0.2 @0.2 @0.2 @0.2 @
    // @Disabling axis 0
    // SELGRIP=1

    CheckIndex( grip, eGID_DIMENSION, "grip" );

    //---------------------
    // ensure sin square velocity profile is used:
    vp( eVP_SIN_SQUARE );
    //---------------------

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 11;
    int nb_lines = sequ ? nb_lines_total : 8;
    //---------------------

    //---------------------
    // send command and parse reply
    Send( cMsg( "selgrip=%d", grip).c_str(), nb_lines, nb_lines_total );

    double T;
    T =   GetDuration( reply[6] )
        + GetDuration( reply[7] );
    //---------------------

    return T;
}
//-----------------------------------------------------------------


double cSDHSerial::grip( double close, double velocity, bool sequ )
    throw (cSDHLibraryException*)
{
    // Actual input/output for the command looks like:
    //--
    // grip=0.1,40
    // @Grip vector is: @0.0 @-66.8 @66.8 @-66.8 @66.8 @-66.8 @66.8 @
    // @Enabling finger axis
    // @Setting current limit to @1.0 @0.5 @0.5 @0.5 @0.5 @0.5 @0.5 @
    // @max distance=8.31, T=0.42s, num_points: 42
    // @Setting current limit to @0.1 @0.2 @0.2 @0.2 @0.2 @0.2 @0.2 @
    // GRIP=0.1

    CheckRange( close, 0.0, 1.0, "close ratio" );
    CheckRange( velocity, 0.0+eps, 100.0, "velocity" );

    //---------------------
    // ensure sin square velocity profile is used:
    vp( eVP_SIN_SQUARE );
    //---------------------

    //---------------------
    // settings for sequential/non-sequential:
    int nb_lines_total = 6;
    int nb_lines = sequ ? nb_lines_total : 4;
    //---------------------

    //---------------------
    // send command and parse reply
    char cmd[] = "grip=CCCCCCCCCCCCCCC,VVVVVVVVVVVVVVV";
    sprintf( cmd, "grip=%f,%f", close, velocity );
    Send( cmd, nb_lines, nb_lines_total );

    double T = GetDuration( reply[3] );
    //---------------------

    return T;
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
