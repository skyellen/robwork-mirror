//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_dsa_cpp_general General file information
  \author   Dirk Osswald
  \date     2008-06-09

  \brief
  This file contains definition of #SDH::cDSA, a class to communicate with the tactile sensors of the %SDH.

  \section sdhlibrary_cpp_dsa_cpp_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

  \subsection sdhlibrary_cpp_dsa_cpp_details SVN related, detailed file specific information:
  $LastChangedBy: Osswald2 $
  $LastChangedDate: 2009-11-02 09:20:24 +0100 (Mo, 02 Nov 2009) $
  \par SVN file revision:
  $Id: dsa.cpp 4932 2009-11-02 08:20:24Z Osswald2 $

  \subsection sdhlibrary_cpp_dsa_cpp_changelog Changelog of this file:
  \include dsa.h.log
*/
//======================================================================

#include "sdhlibrary_settings.h"

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>
#include <iostream>
#include <iomanip>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dsa.h"
#include "sdhbase.h" // for g_sdh_debug_log

//----------------------------------------------------------------------
// Defines, enums, unions, structs
//----------------------------------------------------------------------


USING_NAMESPACE_SDH


//----------------------------------------------------------------------
// Global variables (declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// External functions (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function prototypes (function declarations)
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class declarations
//----------------------------------------------------------------------


void cDSA::WriteCommandWithPayload( UInt8 command, UInt8* payload, UInt16 payload_len )
{
    cCRC_DSACON32m checksum;
#if SDH_USE_VCC
    // VCC cannot create variable size arrays on the stack; so use the heap
    char* buffer = new char[ payload_len+8 ];
#else
    // gcc knows how to allocate variable size arrays on the stack
    char buffer[ payload_len + 8 ]; // 8 = 3 (preamble) + 1 (command) + 2 (len) + 2 (CRC)
#endif
    buffer[0] = (UInt8) 0xaa;
    buffer[1] = (UInt8) 0xaa;
    buffer[2] = (UInt8) 0xaa;
    buffer[3] = command;
    buffer[4] = ((UInt8*) &payload_len)[0];
    buffer[5] = ((UInt8*) &payload_len)[1];

    if ( payload_len > 0)
    {
        // command and payload length are included in checksum (if used)
        checksum.AddByte( command );
        checksum.AddByte( buffer[4] );
        checksum.AddByte( buffer[5] );
    }
    unsigned int i;
    for ( i=0; i < payload_len; i++)
    {
        checksum.AddByte( payload[ i ] );
        buffer[ 6+i ] = payload[i];
    }

    int len;
    if ( payload_len > 0)
    {
        // there is a payload, so the checksum is sent along with the data
        len = payload_len + 8;
        buffer[len-2] = checksum.GetCRC_LB();
        buffer[len-1] = checksum.GetCRC_HB();
    }
    else
    {
        // no payload => no checksum
        len = 6;
    }

    int bytes_written = comm_interface.write( buffer, len );

#if SDH_USE_VCC
    delete[] buffer;
#endif

    if ( bytes_written != len )
        throw new cDSAException( cMsg( "Could only write %d/%d bytes to DSACON32m", bytes_written, len ) );
}
//-----------------------------------------------------------------

void cDSA::ReadResponse( sResponse* response )
{
    assert( response != NULL );

    //---------------------
    // read at most DSA_MAX_PREAMBLE_SEARCH bytes until a valid  preamble 0xaa, 0xaa, 0xaa is found
    UInt16 i;
    UInt8 byte;
    int   nb_preamble_bytes = 0;
    ssize_t bytes_read = 0;
    bool found = false;
    int retries = 0;
    do {
        try {
            bytes_read = comm_interface.Read( &byte, 1, read_timeout_us, false );
        }
        catch ( cRS232Exception* e )
        {
            // ignore timeout
            delete e;
            bytes_read = 0;
        }
        if ( bytes_read == 0 )
            throw new cDSAException( cMsg( "Timeout while reading preamble from remote DSACON32m controller" ) );

        retries++;
        if ( byte == 0xaa )
        {
            nb_preamble_bytes++;
            dbg << "found valid preamble byte no " << nb_preamble_bytes << "\n";
        }
        else
        {
            nb_preamble_bytes = 0;
            dbg << "ignoring invalid preamble byte " << int(byte) << "\n";
        }

        found = nb_preamble_bytes == 3;
    } while ( !found  && retries < DSA_MAX_PREAMBLE_SEARCH );

    if ( !found )
        throw new cDSAException( cMsg( "Could not find valid preamble in %d data bytes from remote DSACON32m controller", bytes_read ) );
    //---------------------

    //---------------------
    // read packet ID and size:
    bytes_read = comm_interface.Read( response, 3, read_timeout_us, false );
    if ( bytes_read != 3 )
    {
        throw new cDSAException( cMsg( "Could only read %d/3 header bytes from remote DSACON32m controller", bytes_read ) );
    }
    //---------------------

    //---------------------
    // check if enough space in payload of response:
    if ( response->payload == NULL  ||  response->max_payload_size < (int) response->size )
    {
        // no, so read and forget the data plus checksum (to keep communication line clean)
        for ( i=0; i < response->size + 2; i++ )
            comm_interface.Read( &byte, 1, 0, true );

        // and report an error
        throw new cDSAException( cMsg( "Not enough space for payload to receive. Only %d bytes buffer space available for %d indicated bytes to receive.", (int) response->max_payload_size, response->size ) );
    }
    //---------------------

    //---------------------
    // read indicated rest (excluding checksum)
    bytes_read = comm_interface.Read( response->payload, response->size, read_timeout_us, false );
    if ( bytes_read != response->size )
    {
        throw new cDSAException( cMsg( "Could only read %d/%d payload bytes from remote DSACON32m controller", bytes_read, response->size ) );
    }
    //---------------------

    //---------------------
    // read and check checksum, if given
    if ( response->size > 0 )
    {
        UInt16         checksum_received;
        cCRC_DSACON32m checksum_calculated;

        bytes_read = comm_interface.Read( (void*) &checksum_received, sizeof( checksum_received ), read_timeout_us, false );
        if ( bytes_read != sizeof( checksum_received ) )
            throw new cDSAException( cMsg( "Could only read %d/2 checksum bytes from remote DSACON32m controller", bytes_read ) );

        checksum_calculated.AddByte( response->packet_id );
        checksum_calculated.AddByte( ((UInt8*) &response->size)[0] );
        checksum_calculated.AddByte( ((UInt8*) &response->size)[1] );
        for ( i=0; i < response->size; i++ )
            checksum_calculated.AddByte( response->payload[i] );

        if ( checksum_received != checksum_calculated.GetCRC() )
            // ??? maybe this should be silently ignored ???
            throw new cDSAException( cMsg( "Checksum Error, got 0x%x but expected 0x%x", checksum_received, checksum_calculated.GetCRC() ) );
        else
            dbg << "Checksum OK\n";
    }
    //---------------------
}
//-----------------------------------------------------------------


void cDSA::ReadControllerInfo( sControllerInfo* _controller_info )
{
    sResponse response( (UInt8*) _controller_info, sizeof( *_controller_info ) );

    ReadResponse( &response );

    // !!! somehow the controller sends only 18 bytes although 19 are expected
    //if ( response.size != sizeof( *_controller_info ) )
    //    throw new cDSAException( cMsg( "Response with controllerinfo has unexpected size %d (expected %d)", response.size, sizeof(*_controller_info) ) );
    if ( 18 != response.size )
        throw new cDSAException( cMsg( "Response with controllerinfo has unexpected size %d (expected %d)", response.size, 18 ) );
}
//-----------------------------------------------------------------


void cDSA::ReadSensorInfo( sSensorInfo* _sensor_info )
{
    sResponse response( (UInt8*) _sensor_info, sizeof( *_sensor_info ) );

    ReadResponse( &response );

    if ( response.size != sizeof( *_sensor_info ) )
        throw new cDSAException( cMsg( "Response with sensorinfo has unexpected size %d (expected %d)", response.size, sizeof(*_sensor_info) ) );
}
//----------------------------------------------------------------------


void cDSA::ReadMatrixInfo( sMatrixInfo* _matrix_info  )
{
    sResponse response( (UInt8*) _matrix_info, sizeof( *_matrix_info ) );

    ReadResponse( &response );

    if ( response.size != sizeof( *_matrix_info ) )
        throw new cDSAException( cMsg( "Response with matrixinfo has unexpected size %d (expected %d)", response.size, sizeof(*_matrix_info) ) );
}
//-----------------------------------------------------------------


void cDSA::ReadFrame( sTactileSensorFrame* frame_p  )
{
    // provide a buffer with space for a full frame without RLE
    // this might fail if RLE is used and the data is not RLE "friendly"
    int buffersize = 4 + nb_cells * sizeof( tTexel );
#if SDH_USE_VCC
    // VCC cannot create variable size arrays on the stack; so use the heap
    UInt8* buffer = new UInt8[ buffersize ];
#else
    // gcc knows how to allocate variable size arrays on the stack
    UInt8 buffer[ buffersize ];
#endif
    sResponse response( buffer, buffersize );
    ReadResponse( &response );

    // size cannot be checked here since it may depend on the data sent (if RLE is used)

    ParseFrame( &response, frame_p );

#if SDH_USE_VCC
    delete[] buffer;
#endif
}
//-----------------------------------------------------------------


void cDSA::QueryControllerInfo( sControllerInfo* _controller_info  )
{
    WriteCommand( 0x01 );
    ReadControllerInfo( _controller_info );
}
//-----------------------------------------------------------------


void cDSA::QuerySensorInfo( sSensorInfo* _sensor_info )
{
    WriteCommand( 0x02 );
    ReadSensorInfo( _sensor_info );
}
//-----------------------------------------------------------------


void cDSA::QueryMatrixInfo( sMatrixInfo* _matrix_info, int matrix_no )
{
    WriteCommandWithPayload( 0x0b, (UInt8*) &matrix_no, 1 );
    ReadMatrixInfo( _matrix_info );
}
//----------------------------------------------------------------------


void cDSA::QueryMatrixInfos( void )
{
    if ( texel_offset != NULL )
    {
        delete[] texel_offset;
        texel_offset = NULL;
    }

    if ( matrix_info != NULL )
    {
        delete[] matrix_info;
        matrix_info = NULL;
    }


    matrix_info = new sMatrixInfo[ sensor_info.nb_matrices ];
    assert( matrix_info != NULL );

    texel_offset = new int[ sensor_info.nb_matrices ];
    assert( texel_offset != NULL );

    nb_cells = 0;

    unsigned int i;
    for ( i=0; i<sensor_info.nb_matrices; i++ )
    {
        texel_offset[i] = nb_cells;
        QueryMatrixInfo( &(matrix_info[i]), i );
        VAR( dbg, matrix_info[i] );

        nb_cells += matrix_info[i].cells_x * matrix_info[i].cells_y;
    }
    VAR( dbg, nb_cells );
}
//-----------------------------------------------------------------


void cDSA::ParseFrame( sResponse* response, sTactileSensorFrame* frame_p )
{
    unsigned int i = 0; // index of next unparsed data byte in payload

    //---------------------
    // copy timestamp and flags from response
    frame_p->timestamp = *(UInt32*) &(response->payload[ i ]);
    i+=4;
    VAR( dbg, frame_p->timestamp );

    frame_p->flags = response->payload[ i ];
    i+=1;
    VAR( dbg, frame_p->flags );

    bool do_RLE = frame_p->flags & (1<<0);
    VAR( dbg, do_RLE );
    //---------------------

    //---------------------
    // for the first frame: record reported timestamp (time of DS) and now (time of pc)
    if (start_dsa == 0)
    {
        start_pc.StoreNow();
        start_dsa = frame_p->timestamp;
        //dbg << "Init start_dsa %d\n" % (start_dsa )
    }
    //---------------------

    if ( dbg.GetFlag() )
    {
        double diff_pc = start_pc.Elapsed();
        UInt32 diff_dsa = frame_p->timestamp - start_dsa;
        dbg.PDM( "ParseFrame: elapsed ms pc,dsa = %6u,%6u  %6u   age %6lu\n", (unsigned int) (diff_pc*1000.0), (unsigned int) diff_dsa, (unsigned int)(((unsigned int)(diff_pc*1000.0))-diff_dsa), GetAgeOfFrame(frame_p) );
    }

    //---------------------
    // copy received data from response to frame
    int j = 0;   // counter for frame elements
    if (do_RLE)
    {
        //---------------------
        // decode RLE encoded frame:

        UInt16 rle_unit;
        tTexel v;
        UInt8  n;
        while (i+1 < response->size)
        {
            rle_unit = *((UInt16*) &(response->payload[ i ]));
            v = rle_unit & 0x0fff;
            n = rle_unit >> 12;
            while (n > 0)
            {
                //response->frame append( v )
                frame_p->texel[ j ] = v;
                n -= 1;
                j += 1;
            }
            i += sizeof( rle_unit );
        }
        if ( j != nb_cells )
            throw new cDSAException( cMsg( "Received RLE encoded frame contains %d texels, but %d are expected", j, nb_cells ) );

        //---------------------
    }
    else
    {
        //---------------------
        // copy non RLE encoded frame:

        if ( response->size - i != (UInt16) (nb_cells * sizeof( tTexel )) )
            throw new cDSAException( cMsg( "Received non RLE encoded frame contains %d bytes, but %d are expected", response->size - i, nb_cells * sizeof( tTexel ) ) );

        memcpy( frame_p->texel, &(response->payload[ i ]), response->size - i );
        //---------------------
    }
}
//-----------------------------------------------------------------


cDSA::cDSA( int debug_level, int port, char const* device_format_string )

    : // init members:
    dbg ( (debug_level>0), "cyan", g_sdh_debug_log ),
    comm_interface ( port, 115200, 1.0, device_format_string ),
    do_RLE (false ),
    matrix_info (NULL),
    frame (),
    nb_cells ( 0 ),
    texel_offset (NULL),
    read_timeout_us (1000000), // 1s
    start_pc (),
    start_dsa (0),
    contact_area_cell_threshold (10),
    contact_force_cell_threshold (10),
    force_factor (1.0),
    calib_pressure (0.000473), // N/(mm*mm)
    calib_voltage (592.1)      // "what the DSA reports:" ~mV

{
    dbg << "Debug messages of class cDSA are printed like this.\n";

    //---------------------
    // check compilation settings (whether the compiler, especially vcc packed structures correctly or not):
    assert( sizeof( sControllerInfo ) == 19 );
    assert( sizeof( sSensorInfo ) == 12 );
    assert( sizeof( sMatrixInfo ) == 52 );
    assert( sizeof( sResponse )   == 7 + sizeof( char* ) );  // on 64 bit systems the char* might be 8 bytes!
    //---------------------

    comm_interface.dbg.SetFlag( debug_level-1 > 0);

    Open();
}
//-----------------------------------------------------------------


void cDSA::Open(void)
{
    comm_interface.Open();

    //---------------------
    // Set framerate of remote DSACON32m to 0 first.
    //   This is necessary since the remote DSACON32m might still be sending frames
    //   from a previous command of another process.
    //   An exception may be thrown if the response for the command gets messed up
    //   with old sent frames, so ignore these.
    //   Additionally we have to ignore the "Detect terminal presence" codes sent by the
    //   DSACON32m after power up / reset. These codes ("\x1b[5n" = 0x1b 0x5b 0x35 0x6e)
    //   are sent 3 times within 0.82 seconds approximately 7 seconds after power up.
    //   The DSACON32m will accept commands only after this time (ca. 8 seconds).
    long old_read_timeout_us = read_timeout_us;
    read_timeout_us = 3000000; // 3000000us = 3s  => 3 retries = 4 tries a 3s = 12s > 8s = power up time of DSACON32m
    SetFramerateRetries( 0, true, true, 3, false );
    read_timeout_us = old_read_timeout_us;
    //---------------------

    //---------------------
    // clean up communication line
    int bytes_read, bytes_read_total = 0;
    long cleanup_timeout_us = 1000000; // start with 1s timeout for first byte
    do {
        UInt8 byte[1001];
        try {
            bytes_read = comm_interface.Read( &byte, 1000, cleanup_timeout_us, true );
        }
        catch ( cRS232Exception* e )
        {
          delete e;
          bytes_read = 0;
          // ignore timeout exception
          break;
        }
        bytes_read_total += bytes_read;
        cleanup_timeout_us = 1000; // following bytes are received with 100ms timeout
    } while (bytes_read > 0);
    dbg << "ignoring " << bytes_read_total << " old bytes of garbage from device\n";
    //---------------------


    //---------------------
    // now query the controller, sensor and matrix info from the remote DSACON32m controller
    QueryControllerInfo( &controller_info );
    VAR( dbg, controller_info );

    QuerySensorInfo( &sensor_info );
    VAR( dbg, sensor_info );

    // read this from data ???
    //sensor_info.bit_resolution = 12;
    //sensor_info.maxvalue = (1 << sensor_info.bit_resolution)-1;

    QueryMatrixInfos();
    //---------------------

    //---------------------
    // now we know the dimension of the attached sensors, so get space for a full frame
    frame.texel = new tTexel[ nb_cells ];
    //---------------------

}
//----------------------------------------------------------------------



cDSA::~cDSA()
{

    if ( frame.texel != NULL )
        delete[] frame.texel;

    if ( texel_offset != NULL )
        delete[] texel_offset;

    if ( matrix_info != NULL )
        delete[] matrix_info;
}
//----------------------------------------------------------------------


void cDSA::Close(void)
{
    dbg << "Closing\n";
    SetFramerateRetries( 0, true, true, 0, true );
    comm_interface.Close();
}
//-----------------------------------------------------------------


void cDSA::SetFramerate( UInt16 framerate, bool do_RLE, bool do_data_acquisition )
{
    dbg << "cDSA::SetFramerate, setting framerate to " << framerate << "\n";
    UInt8 flags = 0;
    if ( do_data_acquisition )
    {
        flags |= (1<<7);
    }

    //if ( do_single_shot )
    //    flags |= (1<<6);

    //if ( do_internal_trigger )
    //    flags |= (1<<5);

    //if ( do_level_trigger )
    //    flags |= (1<<4);

    //if ( do_rising_high )
    //    flags |= (1<<3);

    if ( do_RLE )
    {
        flags |= (1<<0);
    }

    UInt8 buffer[ 3 ];
    buffer[0] = flags;
    buffer[1] = ((UInt8*) &framerate)[0];
    buffer[2] = ((UInt8*) &framerate)[1];

    WriteCommandWithPayload( 0x03, buffer, sizeof( buffer ) );
    sResponse response( buffer, 2 ); // we expect only 2 bytes payload in the answer (the error code)
    ReadResponse( &response );

    if ( response.size != 2 )
    {
        throw new cDSAException( cMsg( "Invalid response from DSACON32m, expected 2 bytes but got %d", response.size ) );
    }
    else if (response.payload[0] != 0  || response.payload[1] != 0   )
    {
        throw new cDSAException( cMsg( "Error response from DSACON32m, errorcode = %d", *((UInt16*) &(response.payload[0])) ) );
    }
    dbg << "acknowledge ok\n";
}
//-----------------------------------------------------------------


void cDSA::SetFramerateRetries( UInt16 framerate, bool do_RLE, bool do_data_acquisition, unsigned int retries, bool ignore_exceptions )
throw (cDSAException*)
{
    do
    {
        try
        {
            SetFramerate( framerate, do_RLE, do_data_acquisition );
            // in case of success we just return
            return;
        }
        catch ( cDSAException* e )
        {
            if ( retries-- == 0  &&  !ignore_exceptions )
                throw; // rethrow in case of retries exceeded
            // catch and ignore exceptions which might result from an invalid response
            dbg << "ignoring Caught exception: " << e->what() << "\n";
            delete e;
        }
        // retry: reopen the interface and try again:
        //comm_interface.Close();
        //comm_interface.Open();
    } while ( retries>0 );
}
//-----------------------------------------------------------------


cDSA::tTexel cDSA::GetTexel( int m, int x, int y ) const
{
    //VAR( dbg,  m );
    //VAR( dbg, sensor_info.nb_matrices );
    assert( 0 <= m  && m < (int) sensor_info.nb_matrices );
    assert( x >= 0  && x < (int) matrix_info[m].cells_x );
    assert( y >= 0  && y < (int) matrix_info[m].cells_y );

    return frame.texel[ texel_offset[m] + y * matrix_info[m].cells_x + x ];
}


//-----------------------------------------------------------------

/*!
    Return contact area in mm*mm for matrix with index \a m
  */
double cDSA::GetContactArea( int m )
{
    double apc  = matrix_info[m].texel_width * matrix_info[m].texel_height; // area per cell
    double area = 0.0;

    for ( int y=0; y < matrix_info[m].cells_y; y++ )
    {
        for ( int x=0; x < matrix_info[m].cells_x; x++ )
        {
            if ( GetTexel( m, x, y ) > contact_area_cell_threshold )
            {
                area += apc;
            }
        }
    }
    return area;
}

//-----------------------------------------------------------------
/*!
    Return a pressure in for cell voltage.
    pressure is in N/(mm*mm)
*/
double cDSA::VoltageToPressure( double voltage )
{
    // !!! for now use linear scale anyway

    /*
    # N/(mm*mm)  <-> Pascal    (from units.exe):
    #   You have: N/(mm*mm)
    #   You want: pascal
    #             * 1000000
    #             / 1e-06
    # => 1 N/(mm*mm) = 1e6 Pascal
    */
    return voltage * calib_pressure / calib_voltage;
}
//-----------------------------------------------------------------


/*!
    Return a sContactInfo struct (force,cog_x,cog_y,area) of contact force and
    center of gravity and contact area of that force for matrix with index \a m
*/
cDSA::sContactInfo cDSA::GetContactInfo( int m )
{
    sContactInfo result;
    double sum_pressures = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    int nbcells = 0;

    for ( int y=0; y < matrix_info[m].cells_y; y++ )
    {
        for ( int x=0; x < matrix_info[m].cells_x; x++ )
        {
            double v = GetTexel( m, x, y );

            if ( v > contact_force_cell_threshold )
            {
                // \attention we cannot just sum up the voltages delivered by the tactile sensor since the
                // correlation between voltage and pressure might be non linear
                // so let calibration_data.VoltageToPressure() handle that
                double p = VoltageToPressure( v );
                sum_pressures += p;
                sum_x += double(x) * p;
                sum_y += double(y) * p;
                nbcells += 1;
            }
        }
    }

    result.area = matrix_info[m].texel_width * matrix_info[m].texel_height * (double) nbcells;

    result.force = force_factor * sum_pressures * result.area;
    if ( sum_pressures != 0.0 )
    {
        result.cog_x = matrix_info[m].texel_width  * sum_x / sum_pressures;
        result.cog_y = matrix_info[m].texel_height * sum_y / sum_pressures;
    }
    else
    {
        result.cog_x = 0.0;
        result.cog_y = 0.0;
    }

    return result;
}
//-----------------------------------------------------------------


#define PRINT_MEMBER( _s, _var, _member )                       \
    (_s) << "  " << #_member << "=" << _var._member << "\n"

#define PRINT_MEMBER_HEX( _s, _var, _member )                       \
    (_s) << "  " << #_member << "=0x" << std::hex << int(_var._member) << std::dec << "\n"


// if the namespace is used then the overloaded operator<< must be explicitly defined in that namespace
#if SDH_USE_NAMESPACE
#define SDH_NAMESPACE_PREFIX  SDH::
#else
#define SDH_NAMESPACE_PREFIX
#endif


std::ostream & SDH_NAMESPACE_PREFIX operator<<( std::ostream &stream,  cDSA::sControllerInfo const &controller_info )
{
    stream << "sControllerInfo:\n";
    PRINT_MEMBER( stream, controller_info, error_code );
    PRINT_MEMBER( stream, controller_info, serial_no );
    PRINT_MEMBER_HEX( stream, controller_info, hw_version );
    PRINT_MEMBER( stream, controller_info, sw_version );
    PRINT_MEMBER_HEX( stream, controller_info, status_flags );
    PRINT_MEMBER_HEX( stream, controller_info, feature_flags );
    PRINT_MEMBER_HEX( stream, controller_info, senscon_type );
    PRINT_MEMBER_HEX( stream, controller_info, active_interface );
    PRINT_MEMBER( stream, controller_info, can_baudrate );
    return PRINT_MEMBER( stream, controller_info, can_id );
}
//----------------------------------------------------------------------


std::ostream & SDH_NAMESPACE_PREFIX operator<<( std::ostream &stream,  cDSA::sSensorInfo const &sensor_info )
{
    stream << "sSensorInfo:\n";
    PRINT_MEMBER( stream, sensor_info, error_code );
    PRINT_MEMBER( stream, sensor_info, nb_matrices );
    PRINT_MEMBER( stream, sensor_info, generated_by );
    PRINT_MEMBER_HEX( stream, sensor_info, hw_revision );
    PRINT_MEMBER( stream, sensor_info, serial_no );
    return PRINT_MEMBER_HEX( stream, sensor_info, feature_flags );
}
//----------------------------------------------------------------------


std::ostream & SDH_NAMESPACE_PREFIX operator<<( std::ostream &stream,  cDSA::sMatrixInfo const &matrix_info )
{
    stream << "sMatrixInfo:\n";
    PRINT_MEMBER( stream, matrix_info, error_code );
    PRINT_MEMBER( stream, matrix_info, texel_width );
    PRINT_MEMBER( stream, matrix_info, texel_height );
    PRINT_MEMBER( stream, matrix_info, cells_x );
    PRINT_MEMBER( stream, matrix_info, cells_y );
    stream << "  " << "uid" << "={";
    unsigned int i;
    for ( i=0; i<(sizeof(matrix_info.uid)/sizeof(matrix_info.uid[0])); i++ )
        stream << " 0x" << std::hex << std::setfill('0') << std::setw(2) << int(matrix_info.uid[i]);
    stream << "}\n";
    stream << "  " << "reserved" << "={";
    for ( i=0; i<(sizeof(matrix_info.reserved)/sizeof(matrix_info.reserved[0])); i++ )
        stream << " 0x" << std::hex << std::setfill('0') << std::setw(2) << int(matrix_info.reserved[i]);
    stream << "}\n" << std::dec << std::setfill(' ');
    PRINT_MEMBER_HEX( stream, matrix_info, hw_revision );

    PRINT_MEMBER( stream, matrix_info, matrix_center_x );
    PRINT_MEMBER( stream, matrix_info, matrix_center_y );
    PRINT_MEMBER( stream, matrix_info, matrix_center_z );

    PRINT_MEMBER( stream, matrix_info, matrix_theta_x );
    PRINT_MEMBER( stream, matrix_info, matrix_theta_y );
    PRINT_MEMBER( stream, matrix_info, matrix_theta_z );
    PRINT_MEMBER( stream, matrix_info, fullscale );
    PRINT_MEMBER_HEX( stream, matrix_info, feature_flags );
    return stream;
}
//----------------------------------------------------------------------

NAMESPACE_SDH_START

std::ostream & operator<<( std::ostream &stream,  cDSA::sResponse const &response )
{
    stream << "sResponse:\n";
    PRINT_MEMBER_HEX( stream, response, packet_id );
    PRINT_MEMBER( stream, response, size );
    PRINT_MEMBER( stream, response, max_payload_size );

    stream << "  payload:\n";
    UInt16 i;
    for ( i=0; i< response.size; i++ )
        stream << "    " << std::setw(3) << i << ": "
               << std::setw(3) << response.payload[i]
               << "0x" << std::hex << response.payload[i];
    stream << std::dec;
    return stream;
}

NAMESPACE_SDH_END
//----------------------------------------------------------------------


std::ostream & SDH_NAMESPACE_PREFIX operator<<( std::ostream &stream,  cDSA const &dsa )
{
    stream << "cDSA.frame:";
    PRINT_MEMBER( stream, dsa.GetFrame(), timestamp );
    PRINT_MEMBER_HEX( stream, dsa.GetFrame(), flags );

    unsigned int m, x, y;
    for ( m = 0; m < dsa.GetSensorInfo().nb_matrices; m++ )
    {
        stream <<  "  matrix " << m << ":\n";

        for ( y = 0; y < dsa.GetMatrixInfo( m ).cells_y; y++ )
        {
            stream << std::setw( 2 ) << y << "| ";
            for ( x = 0; x < dsa.GetMatrixInfo( m ).cells_x; x++ )
            {
                stream << std::setw( 4 ) << dsa.GetTexel( m, x, y ) << " ";
            }
            stream << "\n";
        }
        stream << "\n";
    }
    return stream;
}
//----------------------------------------------------------------------





//======================================================================
/*
  {
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
  }
*/
//======================================================================}
