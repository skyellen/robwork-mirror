//======================================================================
/*!
 \file
 \section sdhlibrary_cpp_canserial_pcan_cpp_general General file information

 \author   Steffen Ruehl, Dirk Osswald
 \date     2009-07-29

 \brief
 Implementation of class #SDH::cCANSerial_PEAK, a class to access a PEAK CAN interface  on cygwin/linux and Visual Studio.
 <HR>
 \internal
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
//#include <termios.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif
//#include <errno.h>
//#include <string.h>
//#include <sys/select.h>
//#include <sys/ioctl.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "canserial-peak.h"
#include "simpletime.h"
#include "util.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

// the Linux version of the PEAK library uses handles,
// while the cygwin/windows version does not. So use a macro
// to keep the code somewhat clearer
#if defined( OSNAME_LINUX )
# define USE_HANDLES( H_ ) (H_),
# define USE_HANDLE( H_ ) (H_)
#else
# define USE_HANDLES( H_ )
# define USE_HANDLE( H_ )
#endif

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;


cCANSerial_PEAK::cCANSerial_PEAK( unsigned long _baudrate, double _timeout, Int32 _id_read, Int32 _id_write, const char *device )
   throw (cCANSerial_PEAKException*)
{
   if ( _timeout < 0.0 )
       throw new cCANSerial_PEAKException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

   handle = (PCAN_HANDLE) NULL;
   baudrate = _baudrate;
   SetTimeout( _timeout );
   id_read = DWORD(_id_read);
   id_write = DWORD(_id_write);
   strncpy(m_device, device, 64);

   ungetch_valid = false;
}
//----------------------------------------------------------------------

cCANSerial_PEAK::cCANSerial_PEAK( PCAN_HANDLE _handle, double _timeout, Int32 _id_read, Int32 _id_write )
   throw (cCANSerial_PEAKException*)
{
   if ( _timeout < 0.0 )
       throw new cCANSerial_PEAKException( cMsg( "Invalid timeout %f (must be >= 0)", _timeout ) );

#if defined( OSNAME_LINUX )
   if ( handle ==  NULL  )
       throw new cCANSerial_PEAKException( cMsg( "Cannot reuse invalid PEAK CAN handle" ) );
#endif

   handle = _handle;
   baudrate = 0;
   SetTimeout( _timeout );
   id_read = DWORD(_id_read);
   id_write = DWORD(_id_write);

   ungetch_valid = false;
}
//----------------------------------------------------------------------

char const* PEAK_strerror( DWORD rc )
{
   switch (rc)
   {
       DEFINE_TO_CASECOMMAND( CAN_ERR_OK);
       DEFINE_TO_CASECOMMAND( CAN_ERR_XMTFULL       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_OVERRUN       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSLIGHT      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSHEAVY      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_BUSOFF        );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QRCVEMPTY     );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QOVERRUN      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_QXMTFULL      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_REGTEST       );
       DEFINE_TO_CASECOMMAND( CAN_ERR_NOVXD         );
       DEFINE_TO_CASECOMMAND( CAN_ERR_RESOURCE      );
       DEFINE_TO_CASECOMMAND( CAN_ERR_ILLPARAMTYPE  );
       DEFINE_TO_CASECOMMAND( CAN_ERR_ILLPARAMVAL   );
   default:
    return "unknown";
   }
}
//-----------------------------------------------------------------


void cCANSerial_PEAK::Open( void )
   throw (cCANSerial_PEAKException*)
{
    DWORD rc;

    if (handle == NULL)
    {
        // only open if we're not reusing an existing handle:

#if defined( OSNAME_LINUX )
        handle = LINUX_CAN_Open((char*) m_device, O_RDWR | O_NONBLOCK);
    	//handle = LINUX_CAN_Open((char*) m_device, O_RDWR );
        if (handle == NULL)
        {
            rc = nGetLastError();
            // open failed, so ensure that handle is invalid
            handle = NULL;
            throw new cCANSerial_PEAKException(cMsg(
                    "Could not open PEAK CAN device \"%s\", rc = 0x%x = \"%s\"", m_device, int(rc),
                    PEAK_strerror(rc)));
        }
#else
        handle = (PCAN_HANDLE) 1; // nothing more to do on cygwin/windows
#endif

        rc = CAN_Init( USE_HANDLES( handle ) BaudrateToBaudrateCode(baudrate), CAN_INIT_TYPE_ST);
        if ( rc )
        {
#if ! defined( OSNAME_LINUX )
            handle = NULL;
#endif
            throw new cCANSerial_PEAKException( cMsg( "Could not set baudrate to %lu on Peak CAN device \"%s\", rc = 0x%x = \"%s\"",
                                                      baudrate, m_device, int(rc), PEAK_strerror(rc)));
        }

        rc = CAN_ResetFilter( USE_HANDLE(handle) );
        if ( rc )
        {
#if ! defined( OSNAME_LINUX )
            handle = NULL;
#endif
            throw new cCANSerial_PEAKException(
                    cMsg(
                            "Could not reset CAN ID 0x%03x on Peak CAN device \"%s\", rc = 0x%x = \"%s\"",
                            (unsigned int) id_read, m_device, int(rc), PEAK_strerror(rc)));
        }

        rc = CAN_MsgFilter( USE_HANDLES(handle) id_write, id_read, MSGTYPE_STANDARD);
        if ( rc )
        {
#if ! defined( OSNAME_LINUX )
            handle = NULL;
#endif
            throw new cCANSerial_PEAKException( cMsg( "Could not add CAN ID 0x%03x on Peak CAN device \"%s\", rc = 0x%x = \"%s\"",
                                                      (unsigned int) id_read, m_device, int(rc), PEAK_strerror(rc)) );
        }
    }
    // (re)init member data:
    M_CMSG_MSG().LEN = 0;
    m_cmsg_next = 0;
}
//----------------------------------------------------------------------


bool cCANSerial_PEAK::IsOpen( void )
   throw()
{
   return ( handle!=NULL );
}
//----------------------------------------------------------------------


void cCANSerial_PEAK::Close( void )
   throw (cCANSerial_PEAKException*)
{
   if ( handle == NULL )
       throw new cCANSerial_PEAKException( cMsg( "Could not close un-opened device" ) );

   CAN_Close( USE_HANDLE( handle ) );
   handle = NULL;
}
//----------------------------------------------------------------------

WORD cCANSerial_PEAK::BaudrateToBaudrateCode( unsigned long baudrate )
   throw (cCANSerial_PEAKException*)
{
   switch (baudrate)
   {
   case 1000000: return CAN_BAUD_1M;
   case 800000:  return CAN_BAUD_500K;
   case 500000:  return CAN_BAUD_500K;
   case 250000:  return CAN_BAUD_250K;
   case 125000:  return CAN_BAUD_125K;
   case 100000:  return CAN_BAUD_100K;
   case 50000:   return CAN_BAUD_50K;
   case 20000:   return CAN_BAUD_20K;
   case 10000:   return CAN_BAUD_10K;
   case 5000:    return CAN_BAUD_5K;
   }

   throw new cCANSerial_PEAKException( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cCANSerial_PEAK::write( char const *ptr, int len )
throw (cCANSerial_PEAKException*)
{
    DWORD rc;

    assert( handle != NULL );

    //cerr << "in cCANSerial_PEAK::write\n"; cerr.flush();
    if ( len == 0 )
        len = int( strlen( ptr ) );

    //cerr << "sending " << len << " bytes <" << ptr << "> to CAN device\n"; cerr.flush();

    // calculate number of CMSGS needed (max 8 data bytes per CMSG)
    Int32 len_cmsgs = len/8 + (((len%8)!=0) ? 1 : 0);

    //---------------------
    TPCANMsg cmsg;
    int j;
    for ( int i=0; i < len_cmsgs; i++)
    {
        // prepare message to send:
        cmsg.ID = id_write;
        cmsg.LEN = min( 8, len-i*8 );
        cmsg.MSGTYPE = MSGTYPE_STANDARD;
        for ( j=0; j<cmsg.LEN; j++ )
            cmsg.DATA[ j ] = *(ptr++);
        //-----

        //-----
        // now send the cmsgs and check return values
#if defined( OSNAME_LINUX )
        rc = LINUX_CAN_Write_Timeout(handle, &cmsg, timeout_us );
#else
        rc = CAN_Write( &cmsg );
#endif
        if ( rc )
        {
            throw new cCANSerial_PEAKException( cMsg( "Could not write message %d/%d on PEAK CAN device \"%s\", rc = 0x%x = \"%s\"", i, (int)len_cmsgs, m_device, int( rc ), PEAK_strerror( rc ) ) );
        }
    }
    return len;
}
//----------------------------------------------------------------------


ssize_t cCANSerial_PEAK::Read( void *_data, ssize_t size, long r_timeout_us, bool return_on_less_data )
throw (cCANSerial_PEAKException*)
{
    DWORD rc;

    assert( handle != NULL );

    char* data = (char*) _data;

#if ! defined( OSNAME_LINUX )
    // in windows (cygwin and VCC) we must implement our own timeout mechanism
    cSimpleTime start_time;
#endif

    //---------------------
    int bytes_read = 0;

    do
    {
        //------
        // first: copy remaining, not yet returned bytes from a previous canRead call to data
        for ( ; m_cmsg_next < M_CMSG_MSG().LEN  &&  bytes_read < size; m_cmsg_next++, bytes_read++ )
            *data++ = M_CMSG_MSG().DATA[ m_cmsg_next ];
        //------

        if ( bytes_read < size )
        {
            //-----
            // if necessary read one more messages with blocking call

            M_CMSG_MSG().LEN = 0;
            m_cmsg_next = 0;
#if defined( OSNAME_LINUX )
            if ( r_timeout_us == 0 )
	      //rc=LINUX_CAN_Read( handle, &m_cmsg ); // will sometimes hang the process here!!!
	        rc=LINUX_CAN_Read_Timeout( handle, &m_cmsg,  0 );
            else
                rc=LINUX_CAN_Read_Timeout( handle, &m_cmsg,  r_timeout_us );
#else
            rc = CAN_Read( &m_cmsg );
#endif
            //-----

            //-----
            // check the received message:

            // simple check: return code of the Read call:
            if (rc==0xFFFFFFFF)
            {
                // ANOTE: although undocumented this seems to be the "no data available" answer when timeout=0

                //std::cerr << "***Ignoring error rc=-1  r_timeout_us=" << r_timeout_us <<"\n";
                continue;
            }

            if (rc > 0)
            {
                M_CMSG_MSG().LEN = 0;

		//if ( (r_timeout_us == 0 || return_on_less_data) &&  rc == CAN_ERR_QRCVEMPTY )
		if ( rc == CAN_ERR_QRCVEMPTY )
                {
		    // no error, just no more data available
#if ! defined( OSNAME_LINUX )
                    if ( return_on_less_data && bytes_read > 0 )
                        return bytes_read;

                    if ( r_timeout_us == 0  || start_time.Elapsed_us() < r_timeout_us)
                        continue;
#endif
                    return bytes_read;
                }
                throw new cCANSerial_PEAKException( cMsg( "Could not read CAN messages from CAN Peak device \"%s\", rc = 0x%x = \"%s\"",  m_device, int( rc ), PEAK_strerror( rc ) ) );
            }

            // check the actual type of the returned message:
            if ( M_CMSG_MSG().MSGTYPE != MSGTYPE_STANDARD )
            {
                M_CMSG_MSG().LEN = 0;
                if ( M_CMSG_MSG().MSGTYPE == MSGTYPE_EXTENDED || M_CMSG_MSG().MSGTYPE == MSGTYPE_RTR )
                {
                    cerr << "Ignoring invalid CAN message of type " << M_CMSG_MSG().MSGTYPE << "\n"; cerr.flush();
                    continue;
                }
                // its a MSGTYPE_STATUS indicating an error
                // so the actual error code is in the data bytes in big endian
                rc = (DWORD(M_CMSG_MSG().DATA[0])<<24) | (DWORD(M_CMSG_MSG().DATA[1])<<16) | (DWORD(M_CMSG_MSG().DATA[2])<<8) | DWORD(M_CMSG_MSG().DATA[3]);
                throw new cCANSerial_PEAKException( cMsg( "Error frame from CAN Peak device \"%s\", rc = 0x%x = \"%s\"",  m_device, int( rc ), PEAK_strerror( rc ) ) );
            }

            // check the ID:
            if ( M_CMSG_MSG().ID != id_read )
            {
                M_CMSG_MSG().LEN = 0;
                throw new cCANSerial_PEAKException( cMsg( "Invalid CAN ID 0x%03x received, expected 0x%03x", (unsigned int) M_CMSG_MSG().ID, (unsigned int) id_read ) );
            }

            // finally copy received bytes to user data:
            for ( ; m_cmsg_next < M_CMSG_MSG().LEN  &&  bytes_read < size; m_cmsg_next++, bytes_read++ )
                *data++ = M_CMSG_MSG().DATA[ m_cmsg_next ];
        }
    } while ( bytes_read < size  &&  !return_on_less_data );


    return bytes_read;
}
//----------------------------------------------------------------------


void cCANSerial_PEAK::SetTimeout( double _timeout )
throw (cSerialBaseException*)
{
    cSerialBase::SetTimeout( _timeout );
#if defined( OSNAME_LINUX )
    timeout_us = int( _timeout * 1E6 );
#endif
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
