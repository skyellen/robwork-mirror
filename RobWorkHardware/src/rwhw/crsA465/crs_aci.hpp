/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWHW_CRSA465_CRS_ACI_HPP
#define RWHW_CRSA465_CRS_ACI_HPP

/**
 * @file crs_aci.hpp
 */

#include <rwhw/serialport/SerialPort.hpp>

#include <cstring>
#include <iostream>

namespace rwhw { namespace crsA465 {

    /** @addtogroup crsA465 */
    /*@{*/


    typedef unsigned long ulong;
    typedef unsigned int uint;
    typedef unsigned short uword;
    typedef short word;
    typedef unsigned char uchar;
    typedef unsigned char ubyte;

    const int MAXRTRY = 3;

    /// @cond EXTERNAL_CODE

    /**
     *
     * Module : IBMACI
     * ------
     *
     * Description :
     * -----------
     *   To provide the IBM relative hardware interface procedures required by the
     *   ACI protocol for execution on an IBM PC/XT/AT.
     *
     *   Hardware specific procedures and static variables - only those needed to
     *   support the hardware implementation. This will be the section that system
     *   programmers will have to customize for individual applications. All procedures
     *   provided here must be duplicated by the system programmer to suit the hardware
     *   implementation. Those procedure declared as 'static', hence local procedures
     *   are provided only to suit the implementation of the required procedures.
     *
     *
     * Global Procedures Defined :
     * -------------------------
     *
     * char get_aci() ** USER MUST PROVIDE **
     *    Actually read in a character. Contains timeout route which will abort
     *    procedure if no character arrives soon enough. Timeout condition must be
     *    signalled by the procedure call 'error_state(0x40);'
     *
     * int aci_rxrdy()
     *    Determine whether or not a character is available in the input ring buffer.
     *
     * void send_aci( out_cha )  ** USER MUST PROVIDE **
     * char out_cha;
     *    Send the specified character to the serial interface. Will timeout if the
     *    interface does not come available within a reasonable length of time.
     *    Timeout condition must be signalled by the procedure call
     *    'error_state(0x50);'
     *
     * int aci_txrdy()
     *    Will determine whether or not the interface is ready to send a byte.
     *
     * inz_aci()                 ** USER MUST PROVIDE **
     * int rate;
     *    Will set the interface to the specified baud rate. Will also initialize the
     *    vector to service the serial receive interrupt. Communication channel will
     *    be ready to go.
     *
     * res_aci()                 ** USER MUST PROVIDE **
     *    This procedure will restore any of the communication interface conditions
     *    to prior-ACI useage. This way, the ACI will be totally disconnected from the
     *    system whenever the ACI has completed.
     *
     * void clean_aci()
     *    Will initialize the input buffer.
     *
     *
     * Global Variables Used :
     * ---------------------
     *  aci_baud
     *  aci_err
     *  aci_channel
     *  aci_slave
     *  aci_pacing
     *  aci_rxto
     *  aci_txto
     *
     * Error Codes :
     * -----------
     *   All ACI shell programs must recognize the possible error codes, and should
     *   determine a safe recovery strategy, depending upon the application. Receive
     *   and transmit character procedures muste support the following error codes
     *   correctly;
     *
     * 40   Timeout on receiving a character.
     * 50   Timeout on Transmitting a character.
     *
     *
     * External Procedures Used :
     * ------------------------
     *  error_state()
     *
     *
     * Revision History :
     * -------+---------+------------------------------------------------------------
     * Version  Date     Description
     * Number
     * -------+---------+------------------------------------------------------------
     * 00      08/05/88  Created
     * 01      11/18/88  Modification made so that multiple communication channels
     *                   could be utilized on the host computer. New global
     *                   aci_channel is required here. com_base[] array, declared
     *                   here then translates the channel number to a base address
     *                   for the communication adaptor.
     *         01/01/90 Modified for Turbo-'C'.
     *                          Utilize turbo-'C' built-in delay() function for millisecond
     *          07/18/91        update_active() calls removed.
     *
     *
     *
     **/


    class IBMACI
    {
    private:
        //      HANDLE aci_com;
        //COMMCONFIG aci_cfig;
        //COMMTIMEOUTS aci_timeouts;

        rwhw::SerialPort m_SerialPort;


    public:

        void clean_aci(void);
        int get_aci_nto(char* c);
        void send_aci(char out_char);
        char get_aci(void);
        void res_aci(void);
        int initrobot(const std::string& port);
        void inz_aci( void );
};

    class ACI00 :public IBMACI
    {
    private:
        word hibyte(word i );
        word lobyte(word i );
        bool string_cmp( char *s1 , char *s2 , word n );
    public:


        void aci_nstrout(char str[] ,word n );
        void aci_strin(char tempstr[] ,word n );
        void enqout(word slave_no );
        void header(uword slave_no ,uword memtype ,uword memofs ,uword memseg ,char rorw ,uword message_length );
        void send_header(uword slave ,uword memtype ,uword memofs ,uword memseg ,char rw ,uword data_length );
        void data_read(word n_bytes ,ubyte data[] );
        void data_out(uword n_bytes ,ubyte data[] );
        void sendenq(uword slave_no );
        void aci_xfr1(uword slave_dev ,uword memtype ,uword memofs ,uword memseg ,char accesstype ,uword bytes ,ubyte* memptr );
        void aci_xfr(uword slave_dev ,uword memtype ,uword memofs ,uword memseg ,char accesstype ,uword bytes ,ubyte* memptr );
        void writerobot(uword offset ,uword segment ,ubyte* srcptr,uword cnt );
        void readrobot(uword offset ,uword segment ,ubyte* destptr,uword cnt );
        void readptrs(ubyte* destptr,word num );
    };

    /*
      ------------------------------------------------------------------------------

      Module : ACI00.C
      ------

      Description :
      -----------
      This module will provide all of the ACI protocol support routines. The
      requirements for the ACI implementation can be divided into four procedure
      categories:
      i)   Hardware Implementation, (the list below specifies which routines must
      be supplied by the programmer for different implementations). These
      procedures are found external to this one.
      ii)  Utility routines,
      iii) Protocol level. These are the procedures which control and enforce the
      ACI protocol standard,
      iv)  Application Interface. These procedures provide quick and easy integration
      of the ACI library to an existing application software package

      Type i) procedures are defined in IBM00.C, or must be provided by the system
      programmer.


      Additional Information
      ----------------------
      The RAPL Technical reference manual, noteably Appendix B discusses the ACI
      protocol in detail. This information is not reproduced here.


      Global Procedures Defined :
      -------------------------

      Utilities
      ---------

      void aci_nstrout( str , n )
      char str[];
      word n;
      Send a string of characters out. The string is pointed to by 'str', and the
      string is 'n' bytes long.

      void aci_strin( tempstr,n )
      char tempstr[];
      word  n;
      Read a string of characters in from the serial interface. The number of characters
      to be expected is 'n', while the destination character array is pointed to
      by 'tempstr'.

      word lobyte(i)
      word i;
      Will return the low byte of the integer.

      hibyte(i)
      word i;
      Will return the hi byte of the integer.

      word string_cmp( s1 , s2 , n )
      byte *s1,*s2;
      word  n;
      Compare two strings, using the specified pointers to the two, and given
      the length of the comparison.


      Protocol
      --------

      aci_xfr( slave_dev,read_write,memofs,memseg,accesstype,bytes,memptr )
      word  slave_dev, read_write, memofs, memseg, bytes;
      byte *memptr;
      char accesstype;
      Main entry point for ACI protcol. All header parameters are entered here.
      Communication channel is initialized before the communication starts, and
      the channel is restored after use. The 'memptr' parameter indicates the
      starting location in the IBM memory for the data transfer. 'Bytes' is a
      word parameter which indicates how many bytes will be transferred. All
      other parameters are explained in the header block description, in the
      Technical manual, Appendix B.

      aci_xfr1( slave_dev,read_write,memofs,memseg,accesstype,bytes,memptr )
      word  slave_dev, read_write, memofs, memseg, bytes;
      byte *memptr;
      char accesstype;
      Main entry point for ACI protcol. All header parameters are entered here.
      The 'memptr' parameter indicates the starting location in the IBM memory
      for the data transfer. 'Bytes' is a word parameter which indicates how
      many bytes will be transferred. All other parameters are explained in the
      header block description, in the Technical manual, Appendix B. Unlike
      aci_xfr(), the communication channel is not initialized or restored after
      use.

      void sendenq( slave_no )
      word  slave_no;
      Initialize the communication with the target robot slave number.

      void data_out( n_bytes,data )
      word  n_bytes;
      byte data[];
      Send data to the slave device. The data is in the data[] array. Send the
      specified number of bytes.

      void data_read( n_bytes,data )
      word  n_bytes;
      byte data[];
      Read the specified number of bytes from the slave. Write it to the data[]
      array.

      void send_header( slave,memtype,memofs,memseg,rw,data_length )
      word  slave, memtype, memofs, memseg, data_length;
      char rw;
      Issue the header to the slave device. Performed after the communication
      is successfully initialized.

      void header( slave_no,memtype,memofs,memseg,rorw,message_length )
      word  slave_no, memtype, memofs, memseg, message_length;
      char rorw;
      Formulate the header block to the slave. See technical description for the
      correct format of the header block.

      void enqout( slave_no )
      word  slave_no;
      Issue the enquiry message to the slave. Expect to read back the correct
      response.

      void error_state( err )
      word err;
      Will set the global error flag 'aci_err'. If the input error
      number is zero, then the error condition will be removed.

      Application Interface Level
      ---------------------------
      These procedures can be interfaced directly by the programmer. Lower level
      procedures are utilized automatically, so the programmer may ignore the ACI
      protocol altogether.

      writerobot( offset , segment , srcptr, cnt )
      byte *srcptr;
      word  offset, cnt, segment;
      Write a buffer of data to the robot controller. The robot controller
      memory is accessed by segment:offset values in standard Intel notation.
      'srcptr' points to the buffer of memory in the computer. 'cnt' is the
      number of bytes to be transferred.

      readrobot( offset , segment , destptr, cnt )
      byte *destptr;
      word  offset, cnt, segment;
      Read a buffer of data from the robot controller. The robot controller
      memory is accessed by segment:offset values in standard Intel notation.
      'destptr' points to the buffer of memory in the computer that will contain
      the image of the robot memory after the procedure has executed. 'cnt' is
      the number of bytes to be transferred.

      NOTE: For both readrobot() and writerobot(), the robot memory is accessed with
      segment:offset parameters. These are integer values. The order of these
      arguments in the argument list makes it possible for the procedures to
      be called as;
      readrobot( robot_ptr, computer_ptr, cnt )
      writerobot( robot_ptr, computer_ptr, cnt )
      char *robot_ptr, *computer_ptr;
      word cnt;
      Where the argument 'robot_ptr' is a proper 4 byte pointer value. This
      convention has been used so that for 'C' compilers that use only 2 byte
      pointers, we may still correctly provide 4 byte pointers to the ACI
      protocol software.

      void readptrs( p , num )
      char *p;
      word num;
      Will return the pointer list, using special hex code 0x40. 'num' refers
      to the number of pointers that will be returned. This procedure always
      returns pointers starting at the top of the list. Each  pointer is a
      standard Intel 4 byte pointer value. 'p' points to the area of computer
      memory which will contain the parameter list after the procedure has
      executed.

      Global Variables Used :
      ---------------------
      word aci_txto;
      Timeout value for transmit character - in msec
      word aci_rxto;
      Timeout value for receive character - in msec
      word aci_baud;
      Baud rate for ACI communication - the defaultis 2400
      word aci_slave;
      Default is slave #1. Can assume a value of 1 to 127
      word  aci_err;
      Global aci error number. Gets set to non-zero if any ACI error occurs.
      When this happens, subsequent ACI programming is aborted.

      Error Codes :
      -----------
      All ACI shell programs must recognize the possible error codes, and should
      determine a safe recovery strategy, depending upon the application.

      04   An enquiry sequence was ignored 32 times.
      12   A header was tried 4 times with no success.
      16   A header was responded to with neither an ACK nor NAK.
      20   A data block read was attempted from the slave 4 times, with a LRC failure each time.
      22   A data block write was attempted to the slave 4 times, with a NAK returning each time.
      24   Invalid response to a data block write  to the slave; neither an ACK nor a NAK returned.
      27   Invalid EOT response from the slave.
      28   Invalid STX read in a data block read from the slave.
      32   Invalid ETB read in a data block read from the slave.
      34   Invalid ETX read in a data block read from the slave.
      40   Timeout on receiving a character.
      50   Timeout on Transmitting a character.
      60   Operator pressed key at some time.
      61   Bad baud rate selection.

      External Procedures Used :
      ------------------------
      These procedures must be provided by the hardware interface level.
      void clean_aci();
      Reset the communication channel
      void send_aci();
      Send a byte to the communication channel
      word inz_aci();
      Initialize the communication channel
      void res_aci();
      Restore the communication channel to the pre-ACI state (if necessary)
      char get_aci();
      Get the next byte from the interface.


      Revision History :
      -------+---------+------------------------------------------------------------
      Version  Date     Description
      Number
      -------+---------+------------------------------------------------------------
      00      07/27/88  Module altered to provide general access to ACI protcol and
      support procedures.
      10/25/88  Included readptrs(), so that the RAPL parameter pointer
      list can be returned
      -------+---------+------------------------------------------------------------
    */


    class ACI : ACI00
    {
    public:
        ACI();
        ~ACI();
        bool Tool(char *loc);
        bool Actual(float &x, float &y, float &z, float &roll, float &pitch, float &yaw);
        bool Abort();
        bool Home();
        bool Ready();
        bool Finish();
        bool Ma(float q1, float q2, float q3, float q4, float q5, float q6);
        bool GetJointConfig(float& q1, float& q2, float& q3, float& q4, float& q5, float& q6);
        bool Move(char *loc);
        bool AsyncMove(char *loc);
        bool SetSpeed(int speed);
        bool KillLocation(char *loc);
        bool CopyLocation(char *loc, char *dest);
        bool SetLocation(char *loc, float x, float y, float z, float roll, float pitch, float yaw);

        bool GetLocation(
            const char *loc,
            float &x,
            float &y,
            float &z,
            float &roll,
            float &pitch,
            float &yaw);

        bool Grip(float pos);
        bool Open(float force);
        bool Close(float force);
        bool Joint(int Index, int Angle);
        bool OpenSession(const std::string& port, int rate, int slave_no = 1);
        void CloseSession();

        bool getSessionOpen() { return sessionopen; };

    private:
        bool sessionopen;
        void garble();
        void swallow();
        bool sendstr(char *s);
        bool recstr(char *buf, int count);
        bool send(char c);
        char get();
    };

    /// @endcond
    /*@}*/

}} // end namespaces

#endif // end include guard
