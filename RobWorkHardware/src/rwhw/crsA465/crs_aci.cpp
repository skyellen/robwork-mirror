#include "crs_aci.hpp"

#include <rw/math/Constants.hpp>

#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#endif

using namespace rwhw::crsA465;
using namespace rw::math;

#ifndef B38400
#define B38400 38400
#endif

#define NO      false
#define YES     TRUE

#define ENQ_ATS    32
#define ENQ        0x05
#define EOT        0x04
#define ETB        0x17
#define ETX        0x03
#define STX        0x02
#define ACK        0x06
#define NAK        0x15
#define SOH        0x01
#define SRCID      0xFF

#define MAXERTRY   32 /* Maximum enquiry attempts */
#define MAXHRTRY   3  /* Maximum header retry limit */
#define MAXDRTRY   3  /* Max data retry limit */
#define N_GET_ATTEMPTS 1000 /*Max number of attempts to call be before timeout */

namespace
{
    uword aci_pacing=0; /* time interval between characters */
    //uword aci_channel=2; /* 1=COM1, 2=COM2 */
    uword aci_txto = 5000; /* Timeout value for transmit character - in msec */
    uword aci_rxto = 5000; /* Timeout value for receive character - in msec */
    uword aci_baud = 38400; /* Baud rate for ACI communication - the default is 2400 */
    uword aci_slave = 1; /* Default is slave #1. Can assume a value of 1 to 127 */

    word aci_err = 0; /* Global aci error number. Gets set to non-zero if any
                       * ACI error occurs. When this happens, subsequent ACI
                       * programming is aborted. */

}

void error_state( word err ) {
    aci_err = err;
}


/*
 * Empty the input buffer, and clear any pending character in the serial input
 * device. This should initialize the input of characters to a known state.
 */
void IBMACI::clean_aci() {
    m_SerialPort.clean();
}


// gets a char if available and returns
int IBMACI::get_aci_nto(char* c) {
    return m_SerialPort.read(c, 1);
}



/*
 * Send a character out the aci interface. CTS must indicate that a slave
 * is available, a transmit timeout would indicate that an interface chip
 * error exists. Error #50 is set if this happens.
 */
void IBMACI::send_aci(char out_char) {
    if (!m_SerialPort.write(&out_char, 1)) {
        std::cout<<"Failed to send char "<<std::endl;
        error_state(0x50);
    }
}



/*
 * Read the next character from the ACI interface. Character must be avaiable to
 * the ACI within the receive timeout limit. If a timeout occurs, error #40 will
 * be set.
 */
char IBMACI::get_aci(void) {
    char ch = 0;
    int attempts = 0;
    while (!m_SerialPort.read(&ch, 1) && attempts<N_GET_ATTEMPTS) {
        attempts++;
#ifdef _WIN32
        Sleep(2);
#else
        usleep(2000);
#endif
    }
    if (attempts>=N_GET_ATTEMPTS) {
        error_state(0x40);
        std::cout<<"No data found in "<<attempts<<" attempts"<<std::endl;
    }
    return ch;
}





/*
 * Restore communication interface
 */
void IBMACI::res_aci(void) {
    /*    _SerialPort.Close();
          if(aci_com)
          CloseHandle(aci_com);
          aci_com = 0;  */
}

int IBMACI::initrobot(const std::string& port) {
    std::cout<<"BaudRate = "<<B38400<<std::endl;

    if (m_SerialPort.open(port, rwhw::SerialPort::Baud38400))
        return true;
    else
        return false;
}



/*
 * Initialize the communication channel, and prepare for talking.
 */
void IBMACI::inz_aci( void ) {
    /*    _SerialPort.Close();
          if (!_SerialPort.Open("/dev/ttyS0",B38400))
          std::cout<<"Could not reopen port "<<std::endl;*/
}





/*************************ACI00 Class*******************************/

/*
 * Return high or low byte in a word quantity
 */
word ACI00::hibyte(word i ) {
    return( i >> 8 );
}


word ACI00::lobyte(word i ) {
    return( i & 0xFF );
}



/*
 * Compare a string of bytes - not a character string, since a NULL can
 * appear anywhere.
 */
bool ACI00::string_cmp( char *s1 , char *s2 , word n ) {
    bool bEqual = true;
    for(int i = 0; i<n && bEqual; i++) {
        if (s1[i] != s2[i]) {
            bEqual = false;
        }
    }
    return bEqual;
}





/**
   ACI system procedures
**/


void ACI00::aci_nstrout(char str[] ,word n ) {
    word  i;
    for( i = 0; i < n; ++i )
        send_aci( str[i] );
}


void ACI00::aci_strin(char tempstr[] ,word n ) {
    word  i;

    for( i = 0; i < n; ++i )
        tempstr[i] = get_aci();
}



/*
 * ACI enquiry and expected response strings. The slave ID number must be
 * encoded into the string before final transmission.
 */

static char enq_string[] = {'R',0,ENQ};

static char slave_response[] = {'R',0,ACK};



/*
 * Issue ENQ sequence to slave controller. Encode slave ID number first.
 */

void ACI00::enqout(word slave_no ) {
    enq_string[1] = slave_no + 0x20;
    aci_nstrout( enq_string , 3 );
}



/*
 * Issue a header block to the slave. All header information must be supplied
 * in the argument list. Checksum is formulated, and the string is sent.
 */
void ACI00::header(uword slave_no ,uword memtype ,uword memofs ,uword memseg ,char rorw ,uword message_length ) {
    int i;
    char rw,lrc = 0;
    char header_string[13];
    uword  lastblk_size,full_blks;

    if ( toupper( rorw ) == 'W' )
        rw = 0x00;
    else
        rw = 0x01;

    full_blks = message_length / 128;
    lastblk_size = message_length % 128;
    if ( lastblk_size == 0 ) {
        --full_blks;
        lastblk_size = 128;
    }

    header_string[0] = SOH;
    header_string[1] = slave_no + 0x20;
    header_string[2] = (char)SRCID;
    header_string[3] = (char)rw;
    header_string[4] = (char)memtype;
    header_string[5] = (char)full_blks;
    header_string[6] = (char)lastblk_size;
    header_string[7] = (char)lobyte( memofs );
    header_string[8] = (char)hibyte( memofs );
    header_string[9] = (char)lobyte( memseg );
    header_string[10] = (char)hibyte( memseg );
    header_string[11] = ETX;

    for( i = 1; i < 11; ++i )
        lrc += header_string[i];

    header_string[12] = (char)lrc;

    aci_nstrout( header_string , 13 );
}





/*
 * Control retry logic for sending header block. Uses header() procedure
 * above to actually send the header data. Controls checking the response from
 * the slave.
 */
void ACI00::send_header(uword slave ,uword memtype ,uword memofs ,uword memseg ,char rw ,uword data_length ) {

    char response;

    word  done,try1;

    for( try1 = 0, done = false ; !done && !aci_err ; ++try1 ) {
        header( slave , memtype , memofs , memseg , rw , data_length );
        response = get_aci();
        if ( !aci_err ) {
            if ( response == ACK ) {
                done = true; /* Done */
            }
            else if ( response == NAK ) {
                if ( try1 == MAXHRTRY ) {
                    error_state( 12 ); /* All retries attempted */
                }
            }
            else {
                error_state( 16 ); /* Improper response from slave */
            }
        }
    }
}





/*
 * Read data from the slave. The specified number of bytes will be dumped
 * into a memory array located by *data. Send the closing EOT character.
 */
void ACI00::data_read(word n_bytes ,ubyte data[] ) {
    word  i, j, k,done;
    uword lastblk_size,full_blks;
    uword tries,index;
    char lrc, in_char;

    /* Establish the number of full and partial block sizes */

    full_blks = n_bytes / 128;

    lastblk_size = n_bytes % 128;

    if ( lastblk_size == 0 ) {
        --full_blks;
        lastblk_size = 128;
    }

    /* Do all full blocks */

    for( index = i = tries = 0 ; i != full_blks && !aci_err ; ) {
        in_char = get_aci();

        if ( !aci_err ) {
            if ( in_char == STX ) {
                for( lrc = 0 , k = i << 7 , j = 0 ; j != 128 && !aci_err ; ) {
                    data[k + j] = get_aci();
                    lrc += data[k + j];
                    ++j;
                }
                if ( !aci_err ) {
                    in_char = get_aci();

                    if ( !aci_err ) {
                        if ( in_char != ETB )
                            error_state( 32 );
                        else {
                            in_char = get_aci();
                            if ( !aci_err ) {
                                if ( in_char == lrc ) {
                                    send_aci( ACK );
                                    ++i;
                                    tries = 0;
                                }
                                else {
                                    send_aci( NAK );
                                    if ( tries++ == MAXDRTRY )
                                        error_state( 20 );
                                }
                            }
                        }
                    }
                }
            }
            else {
                error_state( 28 );
            }
        }
    }

    /* Do the last block */
    for( done = false, tries = 0  ; !done && !aci_err ; ) {
        in_char = get_aci();
        if ( !aci_err ) {
            if ( in_char != STX )
                error_state( 28 );
            else {
                for( i = lrc = 0 , index = full_blks << 7 ; i != lastblk_size && !aci_err ; ++i ) {
                    data[i + index] = get_aci();
                    lrc += data[i + index];
                }
                if ( !aci_err ) {
                    in_char = get_aci();
                    if ( !aci_err ) {
                        if ( in_char != ETX )
                            error_state( 34 );
                        else {
                            in_char = get_aci();
                            if ( !aci_err ) {
                                if ( in_char != lrc ) {
                                    send_aci( NAK );
                                    if ( tries++ == MAXDRTRY )
                                        error_state( 20 );
                                }
                                else {
                                    send_aci( ACK );
                                    done = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if ( !aci_err ) {
        in_char = get_aci();

        if ( in_char != EOT )
            error_state( 27 );

        send_aci( EOT );

    }
}





/*
 * Send data portion to the slave controller. Handle all data block retries.
 * Send the closing EOT character if all goes well or not. We must terminate the
 * sequence regardless.
 */
void ACI00::data_out(uword n_bytes ,ubyte data[] ) {
    char in_char, lrc;
    uword  i, j, tries, k, index,done, full_blks, lastblk_size;

    full_blks = n_bytes / 128;
    lastblk_size = n_bytes % 128;

    if ( lastblk_size == 0 ) {
        --full_blks;
        lastblk_size = 128;
    }

    /* For all full blocks */
    for( done = false , index = i = tries = 0 ; i != full_blks && !aci_err ; ) {
        send_aci( STX );
        for( lrc = 0 , j = 0 , k = i<<7 ; j < 128 ; ++j ) {
            index = k + j;
            send_aci( data[index] );
            lrc += data[index];
        }

        send_aci( ETB );
        send_aci( lrc );
        in_char = get_aci();

        if ( !aci_err ) {
            if ( in_char == ACK ) {
                ++i;
                tries = 0;
            }
            else if ( in_char != NAK )
                error_state( 24 );
            else {
                if ( tries++ == MAXDRTRY )
                    error_state( 22 );
            }
        }
    }

    if ( !aci_err ) {
        for( done = false , tries = 0 ; !done && !aci_err ; ) {
            send_aci( STX );
            for( i = lrc = 0, index = full_blks << 7 ; (i != lastblk_size) ; ++i ) {
                send_aci( data[i + index] );
                lrc += data[i + index];
            }

            send_aci( ETX );
            send_aci( lrc );
            in_char = get_aci();
            if ( !aci_err ) {
                if ( in_char == ACK )
                    done = true;
                else if ( in_char == NAK ) {
                    if ( tries++ == MAXDRTRY )
                        error_state( 22 );
                }
                else {
                    error_state( 24 );
                }
            }
        }
    }

    send_aci( EOT );

}



/*
 * Sends enquiry to slave controller, and analyzes the response
 */

void ACI00::sendenq(uword slave_no )
{
    uword  finished, i;
    char tempstr[3];

    slave_response[1] = slave_no + 0x20;
    for( i = 0, finished = false ; !finished && !aci_err ; ) {
        enqout( slave_no );
        aci_strin( tempstr , 3 );
        if ( !aci_err ) {
            if ( string_cmp( tempstr , slave_response , 3 )) {
                finished = true;
            }
            else if ( i++ == MAXERTRY ) {
                error_state( 4 );
                std::cout<<"Send Enquiry to Robot Failed"<<std::endl;
            }
        }
    }
}





/*
 * Control a complete ACI data transfer. Read and writes are handled here.
 * Complete ACI communication is NOT initialized at the start, and reset at the
 * end of the sequence as in the aci_transfer procedure, since some system
 * initializations send garbage down the serial link, causing slave controller
 * communication aborts. For systems such as these, aci_xfr1() will not re-
 * initialize the interface so it assumes that the interface is in a constant
 * state of readiness. This is probably a good guess is a lot of communication
 * cycles are required all within a short span of time.
 */
void ACI00::aci_xfr1(uword slave_dev ,uword memtype ,uword memofs ,uword memseg ,char accesstype ,uword bytes ,ubyte* memptr ) {
    clean_aci();
    error_state(0); /* Clear all pending errors */

    sendenq( slave_dev );
    if ( !aci_err ) {
        send_header( slave_dev , memtype , memofs , memseg , accesstype , bytes );
        if ( !aci_err ) {
            if ( ( toupper( accesstype ) ) == 'W' ) {
                data_read( bytes , memptr );
            }
            else if (( toupper( accesstype)) == 'R' ) {
                data_out( bytes , memptr );
            }

        }
    }
}





/**
   APPLICATION INTERFACE LEVEL
**/

/*
 * Control a complete ACI data transfer. Read and writes are handled here. Complete
 * ACI communication is initialized at the start, and reset at the end of the
 * sequence.
 */
void ACI00::aci_xfr(uword slave_dev ,uword memtype ,uword memofs ,uword memseg ,char accesstype ,uword bytes ,ubyte* memptr ) {
    inz_aci();
    aci_xfr1( slave_dev , memtype , memofs , memseg , accesstype , bytes , memptr );
    res_aci();
}





/**
 * The following procedures assume that we are talking to the slave identifed by
 * the global slave_device variable. It also assumes that we are accessing
 * standard memory, specified by the '0' memory type qualifier. This is the most
 * convenient robot memory access procedure, as it contains the fewest arguments,
 * but assumes correct setup of the global parameter 'aci_slave'.
 **/



void ACI00::writerobot(uword offset ,uword segment ,ubyte* srcptr,uword cnt ) {
    aci_xfr( aci_slave, 0, offset, segment, 'R', cnt, srcptr );
}





void ACI00::readrobot(uword offset ,uword segment ,ubyte* destptr,uword cnt ) {
    aci_xfr( aci_slave, 0, offset, segment, 'W', cnt, destptr );
}





/**
   This procedure will return the RAPL pointer list contents.
**/


void ACI00::readptrs(ubyte* destptr,word num ) {
    aci_xfr( aci_slave, 0x40, 0, 0, 'W', (num<<2), destptr );
}



/*************************ACI Class*******************************/



ACI::ACI() {
    sessionopen=false;
    std::cout<<"Constructor Session Open = "<<sessionopen<<std::endl;
}





ACI::~ACI() {
    std::cout<<"ACI destructor "<<sessionopen<<std::endl;
    if (sessionopen) {
        std::cout<<"Close Session"<<std::endl;
        CloseSession();
    }
}





bool ACI::Tool(char *loc) {
    char buf[16];

    sprintf(buf, "TOOL %s\r", loc);
    if(!sendstr(buf))
        return false;

    char c;
    while(!get_aci_nto(&c));

    garble();

    return (c == 0x06); // ACK



    /*  if(get()!=0x06)
        return false;   // ACK

        garble();

        return true;*/
}





bool ACI::Home() {
    if(!sendstr("HOME\r"))
        return false;

    char c;

    while(!get_aci_nto(&c));

    garble();
    return (c == 0x06); // ACK
}



bool ACI::Abort() {
    if(!sendstr("ABORT\r")) return false;
    garble();
    return true;
}





bool ACI::Ready() {
    if(!sendstr("READY\r")) return false;

    char c;
    while(!get_aci_nto(&c));

    garble();
    return (c == 0x06); // ACK
}


bool ACI::Finish() {
    if(!sendstr("FINISH\r")) return false;

    char c;
    while(!get_aci_nto(&c));

    garble();
    return (c == 0x06); // ACK
}



bool ACI::SetSpeed(int speed) {
    char buf[11];
    sprintf(buf, "SPEED %d\r", speed);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06)
        return false;   // ACK

    garble();

    return true;

}





bool ACI::Actual(float &x, float &y, float &z, float &roll, float &pitch, float &yaw) {

    if(!sendstr("ACTUAL WORLD\r"))
        return false;

    if(get()!=0x06)  //ACK
        return false;

    garble();

    return GetLocation("WORLD", x, y, z, roll, pitch, yaw);
}





bool ACI::KillLocation(char *loc) {
    char buf[16];

    sprintf(buf, "DLOC %s\r", loc);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;

}





bool ACI::CopyLocation(char *loc, char *dest) {

    char buf[24];

    sprintf(buf, "SET %s=%s\r", dest, loc);
    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;
}



bool ACI::GetLocation(
    const char *loc,
    float &x,
    float &y,
    float &z,
    float &roll,
    float &pitch,
    float &yaw)
{
    char buf[100];

    sprintf(buf, "LLOC %s\r", loc);

    if(!sendstr(buf))
        return false;

    if(!recstr(buf, 72))
        return false;

    if(!recstr(buf, 71))
        return false;

    buf[71] = 0;

    if(sscanf(buf, "%s %f %f %f %f %f %f", buf+80, &x, &y, &z, &roll, &pitch, &yaw) != 7)
        return false;

    x *= (float)Inch2Meter;
    y *= (float)Inch2Meter;
    z *= (float)Inch2Meter;

    roll *= (float)Deg2Rad;
    pitch *= (float)Deg2Rad;
    yaw *= (float)Deg2Rad;

    std::cout<<"New code = "<<std::endl;
    std::cout<<"x = "<<x<<std::endl;
    std::cout<<"y = "<<y<<std::endl;
    std::cout<<"z = "<<z<<std::endl;
    std::cout<<"roll = "<<roll<<std::endl;
    std::cout<<"pitch = "<<pitch<<std::endl;
    std::cout<<"yaw = "<<yaw<<std::endl;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;
}





bool ACI::SetLocation(char *loc, float x, float y, float z, float roll, float pitch, float yaw) {
    char buf[100];

    sprintf(buf, "POINT %s %.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r", loc, x*Meter2Inch, y*Meter2Inch, z*Meter2Inch, roll*Rad2Deg, pitch*Rad2Deg, yaw*Rad2Deg);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;
}


bool ACI::GetJointConfig(float& q1, float& q2, float& q3, float& q4, float& q5, float& q6) {

    if (!sendstr("W2\r"))
        return false;

    char buffer[1200];
    size_t i = 0;

    do {
        buffer[i++] = get();
    }
    while (i<2 || !(buffer[i-2] == '>' && buffer[i-1] == '>'));
    buffer[i] = 0;
    char* jbuf = strstr(buffer, "JOINTS");
    sscanf(jbuf, "%s %f %f %f %f %f %f", buffer, &q1,&q2,&q3,&q4,&q5,&q6);
    q1 *= (float)Deg2Rad;
    q2 *= (float)Deg2Rad;
    q3 *= (float)Deg2Rad;
    q4 *= (float)Deg2Rad;
    q5 *= (float)Deg2Rad;
    q6 *= (float)Deg2Rad;

    return true;

}


bool ACI::Ma(float q1, float q2, float q3, float q4, float q5, float q6) {
    char buf[100];

    sprintf(buf, "MA %.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r", q1, q2, q3, q4, q5, q6);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;

}



bool ACI::AsyncMove(char *loc) {
    char buf[16];

    sprintf(buf, "MOVE %s\r", loc);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06) {   // ACK
        swallow();
        return false;
    }

    garble();

    return true;

}





bool ACI::Move(char *loc) {
    char buf[16];

    sprintf(buf, "MOVE %s\r", loc);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06) {   // ACK
        swallow();
        return false;
    }

    garble();

    if(!sendstr(buf))
        return false;

    char c;
    while(!get_aci_nto(&c));

    garble();

    return (c == 0x06); // ACK

}



bool ACI::Grip(float pos) {
    char buf[26];
    sprintf(buf, "GRIP %f\r", pos);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();

    return true;
}

bool ACI::Joint(int Index, int Angle) {
    char buf[21];
    sprintf(buf, "JOINT %d %d\r", Index, Angle);

    if(!sendstr(buf))
        return false;



    char c;
    while(!get_aci_nto(&c));

    garble();

    return (c == 0x06); // ACK

    /*  if(get()!=0x06) // ACK
        return false;

        garble();
        return true;*/
}

bool ACI::Open(float force) {
    char buf[27];

    sprintf(buf, "OPEN %f\r", force);
    if(!sendstr(buf))
        return false;

    if(get()!=0x06)     // ACK
        return false;

    garble();
    return true;
}

bool ACI::Close(float force) {
    char buf[27];

    sprintf(buf, "CLOSE %f\r", force);

    if(!sendstr(buf))
        return false;

    if(get()!=0x06) // ACK
        return false;

    garble();
    return true;
}




bool ACI::OpenSession(const std::string& port, int rate, int slave_no) {
    char buf[6];
    aci_baud = rate;

    //  aci_channel = port;

    aci_slave = slave_no;
    if(!initrobot(port))
        return false;
    std::cout<<"Init Robot Finished"<<std::endl;
    aci_xfr(aci_slave, 0x48, 0, 0, 'W', 1, (ubyte*)buf);
    if(aci_err) {
        std::cout<<"ACI Error 2"<<std::endl;
        return false;
    }
#ifdef _WIN32
    Sleep(100);
#else
    usleep(100000);
#endif
    send(3);
    garble();

    buf[5] = 0;
    if(!sendstr("NOH"))
        return false;
    recstr(buf, 5);
    std::cout<<"buf = "<<buf<<std::endl;
    send('\r');

    get();
    if(get()!=0x06)
        return false;   // ACK
    garble();

    std::cout<<"Session Open equal true "<<std::endl;
    sessionopen=true;

    return true;

}



void ACI::CloseSession() {
    error_state(0);
    clean_aci();

    if(sessionopen) {
        sendstr("HELP\r");
        get();  // ACK
        garble();
    }

    send(26);
    send(26);
    res_aci();
    std::cout<<"sessionopen = false"<<std::endl;
    sessionopen=false;
}



// garble the robot controller prompt (13, 10, ">>")

void ACI::garble() {
    error_state(0);

    if(get()!= 13)
        return;

    if(get() != 10)
        return;

    if(get()!='>')
        return;

    if(get()!='>')
        return;

}



// swallow any excess message chars returned by controller

void ACI::swallow() {
    error_state(0);
    do {
        get_aci();
    } while(aci_err!=0x40);
}





bool ACI::sendstr(char *s) {
    std::cout<<"Send string "<<s<<std::endl;
    error_state(0);
    bool num = false;

    for(int i=0; i<(int)strlen(s); i++) {
        if(!send(s[i])) {
            std::cout<<"Failed to send string char:"<<s[i]<<std::endl;
            return false;
        }

        if(aci_err || s[i]!=get()) {
            std::cout<<"Failed to get string char:"<<s[i]<<std::endl;
            return false;
        }
#ifdef _WIN32
        Sleep(10);
#else
        usleep(10000);
#endif

        if((s[i]>='0' && s[i]<='9') || s[i]=='.' || s[i]=='-') {
            if(!num) {
                get();
                get();
                get();
                get();
                num = true;
            }
        }
        else num = false;
    }

    return true;

}



bool ACI::recstr(char *buf, int count) {
    error_state(0);

    for(int i=0; i<count; i++) {
        buf[i]=get();
        if(aci_err)
            return false;
    }

    return true;
}





bool ACI::send(char c) {
    error_state(0);
    send_aci(c);
    return !aci_err;
}



char ACI::get() {
    char c = 0;

    error_state(0);

    do {
        c = get_aci();
    } while( c==17 || c==19 );

    return c;
}
