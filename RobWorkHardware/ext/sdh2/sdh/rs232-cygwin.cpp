//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_rs232_cygwin_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-20

  \brief
    Implementation of class #SDH::cRS232, a class to access serial RS232 port on cygwin/linux.

  \section sdhlibrary_cpp_rs232_cygwin_cpp_copyright Copyright

  Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_rs232_cygwin_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2009-08-31 15:46:47 +0200 (Mo, 31 Aug 2009) $
      \par SVN file revision:
        $Id: rs232-cygwin.cpp 4766 2009-08-31 13:46:47Z Osswald2 $

  \subsection sdhlibrary_cpp_rs232_cygwin_cpp_changelog Changelog of this file:
      \include rs232-cygwin.cpp.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#if ! SDH_USE_VCC
# include <unistd.h>
#endif
#include <errno.h>
//#include <string.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <iostream>
#include <exception>
#include <stdarg.h>
#include <cstring>    // needed in gcc-4.3 for prototypes like strcmp according to http://gcc.gnu.org/gcc-4.3/porting_to.html

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "rs232-cygwin.h"
#include "simpletime.h"

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------

/*!
 * Flag, if true then code for debug messages is included.
 *
 * The debug messages must still be enabled at run time by
 * setting the \c some_cRS232_object.dbg.SetFlag(1).
 *
 * This 2 level scheme is used since this is the lowlevel communication,
 * so debug outputs might really steal some performance.
 */
#define SDH_RS232_CYGWIN_DEBUG 1

/*!
 * Flag, if true then ascii codes of data bytes sent/received are printed too
 * if SDH_RS232_CYGWIN_DEBUG is true and the debug level is high enough.
 */
#define SDH_RS232_CYGWIN_DEBUG_ASCII 0


#if SDH_RS232_VCC_DEBUG
# define DBG( ... ) __VA_ARGS__
#else
# define DBG( ... ) ;
#endif

//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function and class member implementation (function definitions)
//----------------------------------------------------------------------

using namespace std;
USING_NAMESPACE_SDH

//! helper function, duplicate string s into a char array allocated with new[]
char* StrDupNew( char* const s )
{
    char* dup = new char[ strlen( s )+1 ];
    return strcpy( dup, s );
}
//----------------------------------------------------------------------

cRS232::cRS232(  int _port, unsigned long _baudrate, double _timeout, char const* _device_format_string )
:   // init base class and members
    cSerialBase(),
    port(_port),
    device_format_string( _device_format_string ),
    baudrate(_baudrate),
    fd(-1),
    status(0)
    // io_set_old cannot be initialized until we have an fd
{
    SetTimeout( _timeout );
}
//----------------------------------------------------------------------

void cRS232::Open( void )
    throw (cRS232Exception*)
{
  char device[ device_format_string.size()+5 ]; // initializer just to get just enough space

  sprintf( device, device_format_string.c_str(), port );
  fd = open( device, O_RDWR | O_NOCTTY );

  if (fd <0)
      throw new cRS232Exception( cMsg( "Could not open device \"%s\", errno = %d = \"%s\"", device, errno, strerror( errno ) ) );

  /*
    c_flags can be values of the following values ORed together
    - CS5, CS6, CS7, or CS8  databits
    - CSTOPB set two stop bits, rather than one.
    - CREAD  enable receiver.
    - PARENB enable  parity  generation  on  output  and  parity checking for input.
    - PARODD parity for input and output is odd.
    - HUPCL  lower modem control lines after last process closes
              the device (hang up).
    - CLOCAL ignore modem control lines
    - B50, B75, B110, B134, B150, B200, B300, B600, B1200,
     B1800, B2400, B4800, B9600, B19200, B38400, B57600,
     B115200, B230400      Baudrates
    - CRTSCTS  flow control.
  */
  unsigned long c_flags = HUPCL | CLOCAL | CREAD| CS8 | BaudrateToBaudrateCode( baudrate );

  termios io_set_new;
  // get device-settings
  if (tcgetattr(fd,&io_set_old) < 0)
  {
    status=errno;
    perror( "cRS232:" );
    return;
  }
  else
    status=0;

  // copy settings from old settings
  io_set_new=io_set_old;

  // declare new settings
  io_set_new.c_cflag = c_flags;
  io_set_new.c_oflag = 0;
  io_set_new.c_iflag = IGNPAR;
  io_set_new.c_lflag = 0;
  io_set_new.c_cc[VMIN] = 1;
  io_set_new.c_cc[VTIME]= 0;
  io_set_new.c_ispeed = BaudrateToBaudrateCode( baudrate );
  io_set_new.c_ospeed = BaudrateToBaudrateCode( baudrate );

  // set new settings
  if (tcsetattr(fd,TCSANOW,&io_set_new) < 0)
  {
    status=errno;
    perror( "cRS232:" );
    return;
  }
  else
    status=0;

}
//----------------------------------------------------------------------


bool cRS232::IsOpen( void )
    throw()
{
  return (fd>=0);
}
//----------------------------------------------------------------------


void cRS232::Close( void )
    throw (cRS232Exception*)
{
  if ( fd < 0 )
    throw new cRS232Exception( cMsg( "Could not close un-opened device" ) );

  close( fd );
  fd = -1;
}
//----------------------------------------------------------------------

tcflag_t cRS232::BaudrateToBaudrateCode( unsigned long baudrate )
    throw (cRS232Exception*)
{
  switch (baudrate)
  {
  case 230400: return B230400;
  case 115200: return B115200;
  case 57600:  return B57600;
  case 38400:  return B38400;
  case 19200:  return B19200;
  case 9600:   return B9600;
  case 4800:   return B4800;
  case 2400:   return B2400;
  case 1800:   return B1800;
  case 1200:   return B1200;
  case 600:    return B600;
  case 300:    return B300;
  case 200:    return B200;
  case 150:    return B150;
  case 134:    return B134;
  case 110:    return B110;
  case 75:     return B75;
  case 50:     return B50;

  }

  throw new cRS232Exception( cMsg( "Invalid baudrate %ld", baudrate ) );
}
//----------------------------------------------------------------------

int cRS232::write( char const *ptr, int len )
    throw (cRS232Exception*)
{
    if ( len == 0 )
        len = strlen( ptr );

    int written = ::write( fd, ptr, len );

#if SDH_RS232_CYGWIN_DEBUG
    dbg << "cRS232::write wrote " << len << "/" << written << " bytes: ";
    int i;
    for ( i = 0; i < len; i++ )
        dbg << int( ((unsigned char*)ptr)[i] ) << ", ";
    dbg << "\n";
#endif

    return written;
}
//----------------------------------------------------------------------


ssize_t cRS232::Read( void *data, ssize_t size, long timeout_us, bool return_on_less_data )
    throw (cRS232Exception*)
{
  if (fd<0)
    return status;

  fd_set fds;
  int bytes_read  = 0;
  int bytes_read_inc;
  int select_return;
  unsigned char *buffer=(unsigned char*)data;

  long max_time_us = timeout_us;
  if (max_time_us<=0) max_time_us=1;
  cSimpleTime start_time;
  timeval time_left;
  long us_left;
  timeval* timeout_p;

  status=0;
  // We wait max max_time_us
  try {
    do
    {
      //---------------------
      // Look for received data with select()

      // calculate time left (min 1 us, otherwise we will read nothing at all)
      if ( max_time_us >= 0)
      {
        us_left = max_time_us - start_time.Elapsed_us();
        time_left.tv_sec  = us_left / 1000000;
        time_left.tv_usec = us_left % 1000000;

        if (time_left.tv_sec <= 0 && time_left.tv_usec < 1 )
        {
          time_left.tv_sec  = 0;
          time_left.tv_usec = 1;
        }

        timeout_p = &time_left;
      }
      else
        timeout_p = NULL; // wait indefinitely



      // prepare select call:
      FD_ZERO(&fds);
      FD_SET(fd, &fds);

      select_return=select(FD_SETSIZE, &fds, 0, 0, timeout_p );

      ////cerr << "select_return = " << select_return <<"\n"; ////
      if ( select_return < 0 )
        // error from select
        throw 4;
      else if (select_return > 0)
      {
        // select says something is available for reading
        if (return_on_less_data)
        {
          // we can return on less, so just read what is available
          bytes_read_inc = read(fd, &buffer[bytes_read], size - bytes_read );
#if SDH_RS232_CYGWIN_DEBUG
          dbg << "cRS232::Read: Read " << bytes_read_inc << "/" << (size-bytes_read) << " bytes: ";
          int i;
          for ( i=0; i<bytes_read_inc; i++)
          {
              dbg << ((unsigned int) (buffer[bytes_read+i]));
#if SDH_RS232_CYGWIN_DEBUG_ASCII
              dbg << "='" << (char) (buffer[bytes_read+i]) << "' ";
#else
              dbg << " ";
#endif
          }
          dbg<< "\n";
#endif

          if (bytes_read_inc < 0)
            // error from read
            throw 1;
          // Any bytes read ?
          if (bytes_read_inc>0)
          {
            bytes_read+=bytes_read_inc;
            if (bytes_read==size)
              return bytes_read;
            //printf("data read:%i\n",bytes_read);
          }
        }
        else
        {
          // we can NOT return on less, so check if enough is available

          //printf("serial:time left %lu\n",Time2Long(tz));
          // Are there already enough bytes received ?

          // from sys/termios.h on cygwin:
          /* TIOCINQ is utilized instead of FIONREAD which has been
             accupied for other purposes under CYGWIN.
             Other UNIX ioctl requests has been omited because
             effects of their work one can achive by standard
             POSIX commands */


          if (ioctl(fd,TIOCINQ,&bytes_read_inc) < 0)
            // error from ioctl
            throw 2;
          else
          {
            // Yes? then read data
            if (bytes_read_inc>=size)
            {
              if ((bytes_read = read(fd, buffer, size )) < 0)
                // error from read
                throw 3;
              else
              {
#if SDH_RS232_CYGWIN_DEBUG
                  dbg << "cRS232::Read: Read " << bytes_read << "/" << size << " bytes: ";
                  int i;
                  for ( i=0; i<bytes_read; i++)
                  {
                      dbg << ((unsigned int) (buffer[i]));
#if SDH_RS232_CYGWIN_DEBUG_ASCII
                      dbg << "='" << (char) (buffer[i]) << "' ";
#else
                      dbg << " ";
#endif
                  }
                  dbg<< "\n";
#endif
                return bytes_read;
              }
            }
            // No ? do nothing
          }
        }
      }
      else
      {
          // (select_return == 0) select returned, but nothing is available

          if (return_on_less_data)
              return bytes_read;

          // else keep trying...
      }
    }
    // Look again for data, if any time left
    while ( timeout_us < 0 || start_time.Elapsed_us() < (long) max_time_us );

    return bytes_read;

  } // end of try block

  catch ( int rc )
  {
    status=errno;
    perror( "Error in rs232::Read: " );

    return status;
  }
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
