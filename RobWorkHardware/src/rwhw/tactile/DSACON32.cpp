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

#include "DSACON32.hpp"

#include <rw/common/macros.hpp>

using namespace rw::common;
using namespace rwhw;

#define DEFAULT_TIMEOUT 1000

#define QUERY_DATA_PACKET 0x00
#define QUERY_CONTROLLER_CONFIG 0x01
#define QUERY_SENSOR_CONFIG 0x02
#define START_DATA_ACQ 0x03
#define READ_MATRIX_MASK 0x04
#define READ_DESCRIPTOR 0x05
#define LOOP_COMMAND 0x06
#define QUERY_CONTROLLER_STATE 0x0A
#define QUERY_MATRIX_CONFIG 0x0B
//#define SET_PROPERTIES_SAMPLE_RATE 0x0C
//#define SET_PROPERTIES_VECTOR_MATRIX 0x0D
//
//#define MATRIX_PROP_CENTROID 0x01
//#define MATRIX_PROP_RESULTING_FORCE 0x02
//#define MATRIX_PROP_CONTACT_AREA_SIZE 0x04
//#define MATRIX_PROP_AVERAGE_FORCE_CONTACT_AREA 0x08
//#define MATRIX_PROP_AVERAGE_FORCE 0x10
//#define MATRIX_PROP_ALL 0x1F

size_t DSACON32::_preambleIdx = 0;

namespace {

    void createData(unsigned char id,
                    unsigned int size,
                    unsigned char data[],
                    int offset = 0)
    {
        data[offset] = 0xAA;
        data[offset+1] = 0xAA;
        data[offset+2] = 0xAA;
        data[offset+3] = id;
        data[offset+4] = size & 0xFF;
        data[offset+5] = (size>>8) & 0xFF;
    }

    void parseMatrixMask(unsigned char payload[], unsigned short payloadSize, TactileMaskMatrix *mask)
    {
        int numX = mask->getMatrix().size1();
        int numY = mask->getMatrix().size2();

        int errorCode = ConvertUtil::toInt16(payload, 0);
        if (errorCode != 0)
            RW_THROW("Parse error: " << errorCode);

        int i = 0;
        int j = 0;
        bool finish = false;
        for(unsigned short n=2; n<payloadSize; n++)
        {
            unsigned short tmp = payload[n];
            for(int d=0; d<8&&!finish; d++)
            {
                mask->set(i, j, tmp & 1);
                tmp = tmp >> 1;
                i++;
                if (i >= numX)
                {
                    i = 0;
                    j++;
                    finish = j >= numY;
                }
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void DSACON32::ControllerConfig::parse(unsigned char payload[])
{
    int errorCode = ConvertUtil::toInt16(payload, 0);
    if (errorCode != 0)
        RW_THROW("Parse error: " << errorCode);
    serialNumber = ConvertUtil::toInt32(payload, 2);
    hwRevision = payload[6];
    swBuild = ConvertUtil::toInt16(payload, 7);
    statusFlags = payload[9];
    featureFlags = payload[10];
    sensconType = (SensconTypes)payload[11];
    interfaceType = (InterfaceTypes)payload[12];
}

std::string DSACON32::ControllerConfig::toString() const
{
    std::ostringstream ostr;
    ostr << "Serial number: " << std::hex << serialNumber << std::endl;
    ostr << "HW revision: " << (hwRevision / 16) << "." << (hwRevision % 16) << std::endl;
    ostr << "SW build: " << swBuild << std::endl;
    ostr << "Status -\n\tsensor ctrl operable: " << ((statusFlags & 128) == 128) << std::endl;
    ostr << "\tdata acquisition running: " << ((statusFlags & 64) == 64) << std::endl;
    ostr << "Features - \n\t USB: " << ((featureFlags & 64) == 64) << std::endl;
    ostr << "\t CAN-bus: " << ((featureFlags & 32) == 32) << std::endl;
    ostr << "\t RS232: " << ((featureFlags & 16) == 16) << std::endl;
    ostr << "Sensor controller type: " <<  sensconType << std::endl;
    ostr << "Current interface: " << interfaceType<< std::endl;

    return ostr.str();
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void DSACON32::SensorConfig::parse(unsigned char payload[])
{
    int errorCode = ConvertUtil::toInt16(payload, 0);
    if (errorCode != 0)
        RW_THROW("Parse error: " << errorCode);
    numMatrices = ConvertUtil::toInt16(payload, 2);
    firmwareRevision = ConvertUtil::toInt16(payload, 4);
    hwRevision = payload[6];
    serialNumber = ConvertUtil::toInt32(payload, 7);
    featureFlags = payload[11];
}

std::string DSACON32::SensorConfig::toString() const
{
    std::ostringstream ostr;
    ostr << "#matrices: " << numMatrices << std::endl;
    ostr << "Firmware rev.: " << firmwareRevision << std::endl;
    ostr << "HW revision: " << (unsigned short)hwRevision << std::endl;
    ostr << "Serial number: " << std::hex << serialNumber << std::endl;
    return ostr.str();
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void DSACON32::MatrixConfig::parse(unsigned char payload[])
{
    int errorCode = ConvertUtil::toInt16(payload, 0);
    if (errorCode != 0)
        RW_THROW("Parse error: " << errorCode);
    texelWidth = ConvertUtil::toFloat32(payload, 2);
    texelHeight = ConvertUtil::toFloat32(payload, 6);
    cellsX = ConvertUtil::toInt16(payload, 10);
    cellsY = ConvertUtil::toInt16(payload, 12);
    uid[2] = ConvertUtil::toInt16(payload, 14);
    uid[1] = ConvertUtil::toInt16(payload, 16);
    uid[0] = ConvertUtil::toInt16(payload, 18);
    hwRevision = payload[22];
    centerX = ConvertUtil::toFloat32(payload, 23);
    centerY = ConvertUtil::toFloat32(payload, 27);
    centerZ = ConvertUtil::toFloat32(payload, 31);
    thetaX = ConvertUtil::toFloat32(payload, 35);
    thetaY = ConvertUtil::toFloat32(payload, 39);
    thetaZ = ConvertUtil::toFloat32(payload, 43);
    fullscale = ConvertUtil::toInt32(payload, 47);
    featureFlags = payload[51];
}

std::string DSACON32::MatrixConfig::toString() const
{
    std::ostringstream ostr;
    ostr << "Texel width:  " << texelWidth << "mm" << std::endl;
    ostr << "Texel height: " << texelHeight << "mm" << std::endl;
    ostr << "#cells x: " << cellsX << std::endl;
    ostr << "#cells y: " << cellsY << std::endl;
    ostr << "UID: " << std::hex << uid[0] << "." << uid[1] << "." << uid[2] << std::endl;
    ostr << "HW revision: " << (hwRevision / 16) << "." << (hwRevision % 16) << std::endl;
    ostr << "Center: (" << centerX << ", " << centerY << ", " << centerZ << ")" << std::endl;
    ostr << "Theta:  (" << thetaX << ", " << thetaY << ", " << thetaZ << ")" << std::endl;
    ostr << "Fullscale: " << fullscale << std::endl;
    ostr << "Feature flags: " << std::hex << featureFlags << std::endl;

    return ostr.str();
}

//////////////////////////////////////////////////////////////////////////////////////////////////


DSACON32::DSACON32(SerialPort &port, const ControllerConfig &cConfig,
                   const SensorConfig &sConfig, MatrixConfig *mConfigs,
                   const std::vector<TactileMaskMatrix> &staticMasks):
    _sPort(&port),
    _useCompression(false),
    _fps(5),
    _cConfig(cConfig),
    _sConfig(sConfig),
    _mConfigs(mConfigs),
    _staticMasks(staticMasks),
//    _isDataAckRunning(false),
    _lastDataAcqTime(0)
{
    for(int i=0; i<_sConfig.numMatrices; i++)
        _pads.push_back(TactileMatrix(_mConfigs[i].cellsX, _mConfigs[i].cellsY));
}

DSACON32::~DSACON32()
{
    delete [] _mConfigs;
}

DSACON32* DSACON32::GetInstance(SerialPort& port)
{
    // If the data acquisition was not stopped in a previous session we will now...
    //SetPropertiesSampleRate(&port, 0x00);
    StopDataAcquisition(&port);
    TimerUtil::sleepMs(1000);
    port.clean();

    // Get information for controller
    ControllerConfig cConfig;
    if(!QueryControllerConfig(&port, &cConfig))
        RW_THROW("Unable to get controller configuration");

    // Get information for sensor
    SensorConfig sConfig;
    if(!QuerySensorConfig(&port, &sConfig))
        RW_THROW("Unable to get controller configuration");

    // Get information for all matrices
    MatrixConfig *mConfigs = new MatrixConfig[sConfig.numMatrices];
    std::vector<TactileMaskMatrix> staticMasks;
    for(unsigned short n=0; n<sConfig.numMatrices; n++)
    {
        if(!QueryMatrixConfig(&port, n, &(mConfigs[n])))
            RW_THROW("Unable to get configuration for matrix #" << n);
        if(!ReadDescriptorString(&port, 0x01, n, &(mConfigs[n].descrStr)))
            RW_THROW("Unable to get descriptor string for matrix #" << n);
        staticMasks.push_back(TactileMaskMatrix(mConfigs[n].cellsX, mConfigs[n].cellsY));
        if(!ReadMatrixMask(&port, 0, n, &staticMasks.back()))
            RW_THROW("Unable to get static mask for matrix #" << n);
    }

    // create DSACON with sensor configuration data
    return new DSACON32(port, cConfig, sConfig, mConfigs, staticMasks);
}

bool DSACON32::queryLoopTest()
{
    unsigned char data[6];
    createData(LOOP_COMMAND,0,data);
    _sPort->write((char*)data, 6);
    return ReadAck(LOOP_COMMAND, DEFAULT_TIMEOUT*2, _sPort);
}

bool DSACON32::queryControllerState()
{
    unsigned char data[6];
    createData(QUERY_CONTROLLER_STATE,0,data);
    _sPort->write((char*)data, 6);
    return ReadAck(QUERY_CONTROLLER_STATE, DEFAULT_TIMEOUT*2, _sPort);
}


bool DSACON32::startDataAcquisition()
{
    if( isDataAcqRunning() )
        return false;

    unsigned char data[100];
    createData(START_DATA_ACQ, 3, data);

    // Flags
    if (_useCompression)
        data[6] = 0xA1;   // With compression
    else
        data[6] = 0xA0;   // Without compression

    // Framerate
    data[7] = _fps & 0xFF;
    data[8] = (_fps>>8) & 0xFF;

    // Checksum
    unsigned short crc = CalcCrc16Checksum(&(data[3]), 6);
    data[9] = crc & 0xFF;
    data[10] = (crc>>8) & 0xFF;

    SendBlocking(data, 11, START_DATA_ACQ, DEFAULT_TIMEOUT, _sPort);

    return true;
}

bool DSACON32::stopDataAcquisition()
{
    return StopDataAcquisition(_sPort);
}

bool DSACON32::readThreadFunc()
{
    unsigned char id;
    unsigned short payloadSize;
    unsigned char header[3];

    if( !ReadHeader(id, payloadSize, header, DEFAULT_TIMEOUT, _sPort) ){
        return false;
    }

    unsigned char *payload = NULL;
    if(payloadSize > 0)
    {
        payload = new unsigned char[payloadSize];
        if( !_sPort->read((char*)payload, payloadSize, DEFAULT_TIMEOUT, 1) )
            return false;
            //RW_THROW("Timeout while recieving payload with id " << (int)cmdId);
    }

    switch(id)
    {
    case QUERY_DATA_PACKET:
        parseDataFrame(payload, payloadSize);
        break;
    //case SET_PROPERTIES_SAMPLE_RATE:
    //  parsePropertiesFrame(payload, payloadSize);
    //  break;
    }

    ValidateChecksum(header, payload, payloadSize, DEFAULT_TIMEOUT, _sPort);
    delete [] payload;

    return true;
}

std::string DSACON32::toString() const
{
    std::ostringstream ostr;
    ostr << "\nController Config:" << std::endl << "---------------------" << std::endl;
    ostr << _cConfig.toString();
    ostr << "\nSensor Config:" << std::endl << "---------------------" << std::endl;
    ostr << _sConfig.toString();
    for(unsigned short n=0; n<_sConfig.numMatrices; n++)
    {
        ostr << "\nMatrix #" << n << ": (" << _mConfigs[n].descrStr << ")" << std::endl << "---------------------" << std::endl;
        ostr << _mConfigs[n].toString();
    }
    return ostr.str();
}

void DSACON32::parseDataFrame(unsigned char *payload, unsigned short payloadSize)
{
    std::vector<TactileMatrix>::iterator pit;
    int time = ConvertUtil::toInt32(payload, 0);
    if (_useCompression)
    {
        pit = _pads.begin();
        int numX = pit->getMatrix().size1();
        int numY = pit->getMatrix().size2();
        int i = 0;
        int j = 0;
        for(int k=11; k<payloadSize; k++)
        {
            unsigned char loByte = payload[k];
            k++;
            unsigned char hiByte = payload[k];
            int count = hiByte >> 4;
            short tmp = (short)(hiByte & 0x0F);
            tmp <<= 8;
            tmp += loByte;
            while (count > 0)
            {
                pit->set(i,j,tmp);
                i++;
                if (i >= numX)
                {
                    i = 0;
                    j++;
                    if (j >= numY)
                    {
                        j = 0;
                        pit++;
                        numX = pit->getMatrix().size1();
                        numY = pit->getMatrix().size2();
                    }
                }
                count--;
            }
        }
    }
    else
    {
        int k = 5;
        for(pit=_pads.begin(); pit!=_pads.end(); pit++)
        {
            int numX = pit->getMatrix().size1();
            int numY = pit->getMatrix().size2();
            for(int j=0; j<numY; j++)
            {
                for(int i=0; i<numX; i++)
                {
                    pit->set(i, j, ConvertUtil::toInt16(payload, k));
                    k += 2;
                }
            }
        }
    }
    _timeStamp = time;
}

void DSACON32::parsePropertiesFrame(unsigned char *payload, unsigned short payloadSize)
{
    int time = ConvertUtil::toInt32(payload, 2);
    std::cout << "parsePropertiesFrame - time: " << time << std::endl;
}

bool DSACON32::StopDataAcquisition(SerialPort *sPort)
{
    unsigned char data[11];
    createData(START_DATA_ACQ, 3, data);

    // Payload Data
    data[6] = 0x00;
    data[7] = 0x00;
    data[8] = 0x00;

    // Checksum
    unsigned short crc = CalcCrc16Checksum(&(data[3]), 6);
    data[9] = crc & 0xFF;
    data[10] = (crc>>8) & 0xFF;

    //SendBlocking(data, 11, START_DATA_ACQ, DEFAULT_TIMEOUT, sPort);
    sPort->write((char*)data, 11);

    return true;

}

bool DSACON32::QueryControllerConfig(SerialPort *sPort, ControllerConfig *cConfig)
{
    unsigned char data[6];
    createData(QUERY_CONTROLLER_CONFIG,0,data);
    sPort->write((char*)data, 6);
    return ReadAck(QUERY_CONTROLLER_CONFIG, DEFAULT_TIMEOUT*2, sPort, cConfig);
}

bool DSACON32::QuerySensorConfig(SerialPort *sPort, SensorConfig *sConfig)
{
    unsigned char data[6];
    createData(QUERY_SENSOR_CONFIG,0,data);
    sPort->write((char*)data, 6);
    return ReadAck(QUERY_SENSOR_CONFIG, DEFAULT_TIMEOUT*2, sPort, sConfig);
}

bool DSACON32::QueryMatrixConfig(SerialPort *sPort, unsigned char matrixNum, MatrixConfig *mConfig)
{
    unsigned char data[9];
    createData(QUERY_MATRIX_CONFIG, 1, data);
    data[6] = matrixNum;
    unsigned short crc = CalcCrc16Checksum(&(data[3]), 4);
    data[7] = crc & 0xFF;
    data[8] = (crc>>8) & 0xFF;
    sPort->write((char*)data, 9);
    return ReadAck(QUERY_MATRIX_CONFIG, DEFAULT_TIMEOUT*2, sPort, mConfig);
}

//bool DSACON32::SetPropertiesSampleRate(rwlibs::io::SerialPort *sPort, unsigned short sampleRate)
//{
//    unsigned char data[10];
//    createData(SET_PROPERTIES_SAMPLE_RATE, 2, data);
//  data[6] = sampleRate & 0xFF;
//  data[7] = (sampleRate>>8) & 0xFF;
//  unsigned short crc = CalcCrc16Checksum(&(data[3]), 5);
//  data[8] = crc & 0xFF;
//  data[9] = (crc>>8) & 0xFF;
//    sPort->write((char*)data, 10);
//  return ReadAck(SET_PROPERTIES_SAMPLE_RATE, DEFAULT_TIMEOUT*2, sPort);
//}
//
//bool DSACON32::SetPropertiesVectorMatrix(rwlibs::io::SerialPort *sPort, unsigned char matrixNum,
//                                       unsigned char bitVector)
//{
//    unsigned char data[10];
//    createData(SET_PROPERTIES_VECTOR_MATRIX, 2, data);
//  data[6] = matrixNum;
//  data[7] = bitVector;
//  unsigned short crc = CalcCrc16Checksum(&(data[3]), 5);
//  data[8] = crc & 0xFF;
//  data[9] = (crc>>8) & 0xFF;
//    sPort->write((char*)data, 10);
//  return ReadAck(SET_PROPERTIES_VECTOR_MATRIX, DEFAULT_TIMEOUT*2, sPort);
//}


bool DSACON32::ReadMatrixMask(SerialPort *sPort, unsigned char maskType,
                              unsigned char matrixNum, TactileMaskMatrix *mat)
{
    unsigned char data[10];
    createData(READ_MATRIX_MASK, 2, data);
    data[6] = maskType;         // Mask type 0x00=static, 0x01=dynamic
    data[7] = matrixNum;        // Index of matrix
    unsigned short crc = CalcCrc16Checksum(&(data[3]), 5);
    data[8] = crc & 0xFF;
    data[9] = (crc>>8) & 0xFF;
    sPort->write((char*)data, 10);
    return ReadAck(READ_MATRIX_MASK, DEFAULT_TIMEOUT*2, sPort, mat);
}

bool DSACON32::ReadDescriptorString(SerialPort *sPort, unsigned char descrType,
                                    unsigned char matrixNum, std::string *str)
{
    unsigned char data[10];
    createData(READ_DESCRIPTOR, 2, data);
    data[6] = descrType;        // Descriptor type 0x00=sensor, 0x01=matrix
    data[7] = matrixNum;        // Index of matrix (only if descriptor type is: 0x01)
    unsigned short crc = CalcCrc16Checksum(&(data[3]), 5);
    data[8] = crc & 0xFF;
    data[9] = (crc>>8) & 0xFF;
    sPort->write((char*)data, 10);
    return ReadAck(READ_DESCRIPTOR, DEFAULT_TIMEOUT*2, sPort, str);
}

bool DSACON32::ReadAck(unsigned char cmdId, unsigned int timeout, SerialPort *sPort, void* answer)
{
    unsigned char id;
    unsigned short payloadSize;
    unsigned char header[3];

    if( !ReadHeader(id, payloadSize, header, timeout, sPort) )
        RW_THROW("Timeout while recieving header with id " << (int)cmdId);
    if( id != cmdId )
        RW_WARN("Wrong packet id recieved! Got: " << (int)id << ", wanted: " << (int)cmdId);

    unsigned char *payload = NULL;
    if(payloadSize > 0)
    {
        payload = new unsigned char[payloadSize];
        if( !sPort->read((char*)payload, payloadSize, timeout, 1) )
            RW_THROW("Timeout while recieving payload with id " << (int)cmdId);
    }
    ParsePacket(payload, payloadSize, id, answer);
    ValidateChecksum(header, payload, payloadSize, timeout, sPort);
    delete [] payload;
    return id==cmdId;
}

bool DSACON32::ReadHeader(unsigned char &id, unsigned short &payloadSize, unsigned char *header,
                          unsigned int timeout, SerialPort *sPort)
{
    // Search for preamble
    do
    {
        if( !sPort->read((char*)header, 1, timeout, 1) )
            return false;

        if(header[0] == 0xAA)
            _preambleIdx++;
        else
            _preambleIdx = 0;
    }
    while(_preambleIdx < 3);
    // Remember to reset preamble index
    _preambleIdx=0;

    // Now read packet id [0] and payload size [1:2]
    if( !sPort->read((char*)header, 3, timeout, 1) )
        return false;

    // Save id
    id = header[0];
    // Save payload
    payloadSize = ConvertUtil::toInt16(header, 1);

    return true;
}

void DSACON32::ParsePacket(unsigned char *payload, unsigned short payloadSize, unsigned char id, void* answer)
{
    int errorCode;
    switch (id)
    {
    case QUERY_DATA_PACKET:      // Data frame
        break;
    case QUERY_CONTROLLER_CONFIG:
        static_cast<ControllerConfig*>(answer)->parse(payload);
        break;
    case QUERY_SENSOR_CONFIG:
        static_cast<SensorConfig*>(answer)->parse(payload);
        break;
    case QUERY_MATRIX_CONFIG:
        static_cast<MatrixConfig*>(answer)->parse(payload);
        break;
    case READ_MATRIX_MASK:
        parseMatrixMask(payload, payloadSize, static_cast<TactileMaskMatrix*>(answer));
        break;
    case READ_DESCRIPTOR:
        errorCode = ConvertUtil::toInt16(payload, 0);
        if (errorCode != 0)
            RW_THROW("READ_DESCRIPTOR error code=: " << errorCode);
        *static_cast<std::string*>(answer) = payloadSize>2 ? std::string((char*)&(payload[2]), payloadSize-2) : "N/A";
        break;
    case START_DATA_ACQ:      // Data acquisition
        //errorCode = ConvertUtil::toInt16(data, 6);
        //if (errorCode != 0)
        //    RW_THROW("Parse error: " << errorCode);
        //_isDataAckRunning = !_isDataAckRunning;
        break;
    case LOOP_COMMAND:      // Loop command
        break;
    case QUERY_CONTROLLER_STATE:      // Controller state
        errorCode = ConvertUtil::toInt16(payload, 0);
        if (errorCode != 0)
            RW_THROW("QUERY_CONTROLLER_STATE error code=: " << errorCode);
        //OnCtrlState(errorCode);
        break;
    //case SET_PROPERTIES_SAMPLE_RATE:
    //  break;
    //case SET_PROPERTIES_VECTOR_MATRIX:
    //  break;
    default:
        RW_THROW("BAD ID RECIEVED" << (int) id);
    }
}

void DSACON32::ValidateChecksum(unsigned char *header, unsigned char *payload,
                                unsigned short payloadSize, unsigned int timeout,
                                SerialPort *sPort)
{
    unsigned short crc_calc = CalcCrc16Checksum(header, 3);
    crc_calc = CalcCrc16Checksum(payload, payloadSize, crc_calc);

    unsigned char buffer[2];
    if( !sPort->read((char *) buffer, 2, timeout, 1) )
        RW_THROW("Timeout while reading checksum");
    unsigned short crc_read = ConvertUtil::toInt16(buffer, 0);

    if(crc_calc != crc_read)
        RW_THROW("DSACON32::ValidateChecksum - Checksum error");
}

unsigned short DSACON32::CalcCrc16Checksum(unsigned char *data, unsigned short size, unsigned short crc)
{
    unsigned long c;
    /* process each byte prior to checksum field */
    for ( c=0; c < size; c++ )
    {
        crc = CRC_TABLE_CCITT16[ ( crc ^ *( data ++ )) & 0x00FF ] ^ ( crc >> 8 );
    }
    return crc;
}

const unsigned short DSACON32::CRC_TABLE_CCITT16[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};
