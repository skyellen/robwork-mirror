//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_vcc_cpp_general General file information
    \author   Martin
    \date     2008-05-23


  \brief
    Implementation of class #SDH::cRS232, a class to access serial RS232 port with VCC compiler on Windows.

  \section sdhlibrary_cpp_rs232_vcc_cpp_cpp_copyright Copyright
    Code kindly provided by Martin from the RoboCluster project Denmark.

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_vcc_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-16 18:59:36 +0200 (Do, 16 Okt 2008) $
      \par SVN file revision:
        $Id: rs232-vcc.cpp 3722 2008-10-16 16:59:36Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_vcc_cpp_changelog Changelog of this file:
      \include rs232-vcc.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include "iostream"

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#define _CRT_SECURE_NO_WARNINGS 1

#include "rs232-vcc.h"
#include "simpletime.h"
#include "sdhlibrary_settings.h"
#include "util.h"
#include "dbg.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------

USING_NAMESPACE_SDH

//! flag, if 1 then debug messages are enabled here in rs232-vcc.cpp. Else messages are disabled without any overhead.
#define SDH_RS232_VCC_DEBUG 0

#if SDH_RS232_VCC_DEBUG
# define DBG( ... ) __VA_ARGS__
#else
# define DBG( ... ) ;
#endif

//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------

NAMESPACE_SDH_START

//! a debug object to print low level rs232 messages
DBG( cDBG g_rs232_dbg( SDH_RS232_VCC_DEBUG, "blue" ); )

NAMESPACE_SDH_END

//----------------------------------------------------------------------
// External functions and classes (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function definitions
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class member definitions
//----------------------------------------------------------------------


cRS232::cRS232(  int _port, unsigned long _baudrate, double _timeout )
#ifndef RS_232_TEST
    : _hCOM(NULL)
#endif
{
    port = _port;
    baudrate = _baudrate;
    SetTimeout( _timeout );
    ungetch_valid = false;
}

cRS232::~cRS232(void)
{
    if (_hCOM)
      Close();
}

#ifndef RS_232_TEST

void cRS232::Open( void )
    throw (cRS232Exception*)
{
    // see e.g. http://msdn.microsoft.com/de-de/magazine/cc301786(en-us).aspx
    //
    char device[] = "\\\\.\\COM00"; // initializer just to get just enough space
    sprintf(device, "\\\\.\\COM%d", port+1);
    _hCOM = CreateFileA(device,                        // lpFileName
                        GENERIC_READ | GENERIC_WRITE,  // dwDesiredAccess
                        0,                             // dwShareMode
                        0,                             // lpSecurityAttributes
                        OPEN_EXISTING,                 // dwCreationDisposition
#if SDH_RS232_VCC_ASYNC
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, // dwFlagsAndAttributes
#else
                        FILE_ATTRIBUTE_NORMAL,         // dwFlagsAndAttributes
#endif
                        0);                            // hTemplateFile
#if SDH_RS232_VCC_ASYNC
    memset(&o, 0, sizeof(OVERLAPPED));
#endif
    DCB dcbInitState;
    GetCommState(_hCOM, &dcbInitState);
    dcbInitState.BaudRate = baudrate;
    dcbInitState.fOutxCtsFlow = 0;
    dcbInitState.fDtrControl = DTR_CONTROL_DISABLE;
    dcbInitState.fRtsControl = RTS_CONTROL_DISABLE;
    dcbInitState.StopBits = ONESTOPBIT;
    SetCommState(_hCOM, &dcbInitState);
    SleepSec(0.060);

    BOOL rc = GetCommTimeouts( _hCOM, &comm_timeouts );
    if ( rc == 0 )
        throw new cRS232Exception( cMsg( "Could not read timeouts of RS232 port %d = COM%d", port, port+1 ) );

    SetTimeout( timeout );
}

void cRS232::Close()
    throw (cRS232Exception*)
{
    CloseHandle(_hCOM);
    _hCOM = NULL;
}

void cRS232::SetTimeout( double _timeout )
    throw (cSerialBaseException*)
{
  DBG( g_rs232_dbg << "setting timeout to " << _timeout << "\n"; )
  if ( _hCOM != NULL )
  {
      // we have a valid handle, so set the new timeout
      // see http://msdn.microsoft.com/en-us/library/aa363194(VS.85).aspx

      DWORD ms = DWORD( _timeout * 1000.0 ); // new timeout in ms

      comm_timeouts.ReadIntervalTimeout        = 0; //MAXDWORD;
      comm_timeouts.ReadTotalTimeoutMultiplier = 0;
      comm_timeouts.ReadTotalTimeoutConstant   = ms; // set the timeout for the total read only

      BOOL rc = SetCommTimeouts( _hCOM, &comm_timeouts );
      if ( !rc )
          throw new cRS232Exception( cMsg( "Could not set timeouts of RS232 port %d = COM%d", port, port+1 ) );
  }
  read_timeout_us = long( _timeout * 1000000.0 );
  cSerialBase::SetTimeout( _timeout );
}

ssize_t cRS232::Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
    throw (cRS232Exception*)
{
    double old_timeout = timeout;
    if ( timeout_us != read_timeout_us )
        SetTimeout( double( timeout_us ) / 1000000.0 );  // TODO: converting back and forth is very inefficient!

    char* datap = (char*) data;
    memset(datap, 0, size*sizeof(char));
    DWORD offset=0;
    DWORD bytes_read;
    do
    {
#if SDH_RS232_VCC_ASYNC
        memset(&o, 0, sizeof(OVERLAPPED));
        o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        if (!ReadFile(_hCOM, &datap[offset], 1, &bytes_read, &o))
#else
        if (!ReadFile(_hCOM, &datap[offset], 1, &bytes_read, NULL )) //async , &o))
#endif
        {
#if SDH_RS232_VCC_ASYNC
            DWORD last_error = GetLastError();
            if ( last_error == ERROR_IO_PENDING )
            {
                DWORD nRetVal;
                if(return_on_less_data)
                    nRetVal = WaitForSingleObject(o.hEvent, 10);
                else
                    nRetVal = WaitForSingleObject(o.hEvent, 5000);
                switch(nRetVal)
                {
                case WAIT_OBJECT_0:             // ReadFile event
                    break;
                case WAIT_TIMEOUT:
                    DBG( g_rs232_dbg << "WAIT_TIMEOUT bytes_read = " << bytes_read << "\n"; )
                    CloseHandle(o.hEvent);
                    throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
                }
            }
            else
            {
              std::cerr << "error " << last_error << " from ReadFile / GetLastError\n";
            }
#else
          throw new cRS232Exception( cMsg( "ReadFile error Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
#endif
        }
        if ( bytes_read == 0 )
        {
             DBG( g_rs232_dbg << "timeout bytes_read = " << bytes_read << "\n"; )
#if SDH_RS232_VCC_ASYNC
             CloseHandle(o.hEvent);
#endif
             throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
        }
#if SDH_RS232_VCC_ASYNC
        CloseHandle(o.hEvent);
#endif
        offset += bytes_read;
        //if(data[offset-1] != eol[0])
        //    SleepSec(0.001);
    } while( offset < (DWORD) size );

    if ( old_timeout != timeout )
        SetTimeout( old_timeout );  // TODO: setting back and forth is very inefficient!

    DBG( g_rs232_dbg << "read " << bytes_read << " bytes with timeout " << timeout_us << " us: "; )

#if SDH_RS232_VCC_DEBUG
    DWORD i;
    for ( i = 0; i < bytes_read; i++ )
      DBG( g_rs232_dbg << int( ((unsigned char*)data)[i] ) << ", "; )
    DBG( g_rs232_dbg << "\n"; )
#endif

    return (ssize_t) offset;
}

char* cRS232::readline(char* line, int size, char* eol, bool return_on_less_data)
    throw (cRS232Exception*)
{
    memset(line, 0, size*sizeof(char));
    DWORD offset=0;
    DWORD bytes_read;
    do
    {
#if SDH_RS232_VCC_ASYNC
        memset(&o, 0, sizeof(OVERLAPPED));
        o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
        if (!ReadFile(_hCOM, &line[offset], 1, &bytes_read, &o))
#else
        if (!ReadFile(_hCOM, &line[offset], 1, &bytes_read, NULL))
#endif
        {
#if SDH_RS232_VCC_ASYNC
            DWORD last_error = GetLastError();
            if ( last_error == ERROR_IO_PENDING )
            {
                DWORD nRetVal;
                if(return_on_less_data)
                    nRetVal = WaitForSingleObject(o.hEvent, 10);
                else
                    nRetVal = WaitForSingleObject(o.hEvent, 5000);
                switch(nRetVal)
                {
                case WAIT_OBJECT_0:             // ReadFile event
                    break;
                case WAIT_TIMEOUT:
                    throw new cRS232Exception( cMsg( "Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
                }
            }
            else
            {
              std::cerr << "error " << last_error << " from ReadFile / GetLastError\n";
            }
#else
            throw new cRS232Exception( cMsg( "ReadFile Timeout while reading data from RS232 port %d = COM%d", port, port+1 ) );
#endif
        }
#if SDH_RS232_VCC_ASYNC
        CloseHandle(o.hEvent);
#endif
        offset += bytes_read;
        //if(line[offset-1] != eol[0])
        //    SleepSec(0.001);
    } while(line[offset-1] != eol[0]);

    return line;
}

int cRS232::write(char const *ptr, int len)
    throw (cRS232Exception*)
{
    if(len == 0)
        len = static_cast<int>(strlen(ptr));

    DWORD dwWritten;
#if SDH_RS232_VCC_ASYNC
    //OVERLAPPED o = {0};
    o.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);
    WriteFile(_hCOM, (LPCVOID)ptr, len, &dwWritten, &o);
#else
    WriteFile(_hCOM, (LPCVOID)ptr, len, &dwWritten, NULL );
#endif

#if SDH_RS232_VCC_DEBUG
    DBG( g_rs232_dbg << "cRS232::write wrote " << len << "/" << dwWritten << " bytes: "; )
    int i;
    for ( i = 0; i < len; i++ )
      DBG( g_rs232_dbg << int( ((unsigned char*)ptr)[i] ) << ", "; )
    DBG( g_rs232_dbg << "\n"; )
#endif

    //!!! dwWritten is always 0! Damn bloody windows
    //if(dwWritten != len)
    //    throw new cRS232Exception( cMsg( "Unable to write %d bytes to port %d = COM%d (only %d written)", len, port, port+1, dwWritten ) );

    //return dwWritten;
    return len;
}

#else

void cRS232::Open(int port, unsigned long baudrate, double timeout)
    throw (cRS232Exception*)
{
    _timeout = timeout;
}

void cRS232::Close()
    throw (cRS232Exception*)
{
}

char* cRS232::readline(char* line, int size, char* eol, bool return_on_less_data)
    throw (cRS232Exception*)
{
    std::cout << "EOL size=" << strlen(eol) << std::endl;
    if(return_on_less_data)
        throw new cRS232Exception("return_on_less_data");

    return "cRS232::readline";
}

int cRS232::write(char const *ptr, int len)
    throw (cRS232Exception*)
{
    if(len == 0)
        len = strlen( ptr );
    std::cout << ">>> " << ptr << std::endl;
    return len;
}

#endif
