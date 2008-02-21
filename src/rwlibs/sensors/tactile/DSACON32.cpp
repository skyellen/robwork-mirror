#include "DSACON32.hpp"


#include <rw/common/ConvertUtil.hpp>
#include <rw/common/macros.hpp>

using namespace rw::common;
using namespace rwlibs::io;
using namespace rwlibs::sensors;

#define DEFAULT_TIMEOUT 1000

#define QUERY_DATA_PACKET 0x00
#define QUERY_CONTROLLER_CONFIG 0x01
#define QUERY_SENSOR_CONFIG 0x02
#define START_DATA_ACQ 0x03
#define READ_DESCRIPTOR 0x05
#define LOOP_COMMAND 0x06
#define QUERY_CONTROLLER_STATE 0x0A

namespace {
    
    void createData(unsigned char id, 
                    unsigned int size, 
                    unsigned char data[],
                    int offset = 0){
        data[offset] = 0xAA;
        data[offset+1] = 0xAA;
        data[offset+2] = 0xAA;
        data[offset+3] = id;
        data[offset+4] = size & 0xFF;
        data[offset+5] = (size>>8) & 0xFF;
        /*std::cout << "Package Head: [" 
        	      << std::hex << (unsigned int)data[offset] << "," 
        	      << (unsigned int)data[offset+1] << ","
        	      << (unsigned int)data[offset+2] << ","
        	      << (unsigned int)data[offset+3] << ","
        	      << (unsigned int)data[offset+4] << ","
        	      << (unsigned int)data[offset+5] << "]" <<std::endl;
         */
    }
    
    void validateChecksum(unsigned char data[], int checksumIndx)
    {
        unsigned char checksum = 0;
        for (int i = 6; i < checksumIndx; i++)
            checksum += data[i];
        if (checksum != data[checksumIndx])
            RW_THROW("DSACON32::ValidateChecksum - Checksum error");
    }

    void parseSensorCfg(unsigned char *data, DSACON32::SensorConfig &sConfig)
    {
        int errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_THROW("Parse error: " << errorCode);
        sConfig.parse(data);
        validateChecksum(data, 27);
    }

    void parseControllerCfg(unsigned char *data, DSACON32::ControllerConfig &config)
    {
        int errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_THROW("Parse error: " << errorCode);
        config.parse(data);
        validateChecksum(data, 25);
    }
}

DSACON32::DSACON32(SerialPort &port, 
					const ControllerConfig &config, 
					const SensorConfig &sConfig):
    _sPort(&port),
    _padA( sConfig.cellsX , sConfig.cellsY/2 ),
    _padB( sConfig.cellsX , sConfig.cellsY/2 ),
    _useCompression( false ),
    _fps(5),
    _isDataAckRunning(false),
    _cConfig(config),
    _sConfig(sConfig),
    _lastDataAcqTime(0)
{
    
}

DSACON32::~DSACON32()
{
}

DSACON32* DSACON32::GetInstance(SerialPort& port) {
    // detect if any DSACON32 unit is connected to this serialPort
	//std::cout << "Query Loop!!" << std::endl;	
	QueryLoopTest(port);
    // get configuration
    //std::cout << "Query Controller config!!" << std::endl;
    ControllerConfig config;
    QueryControllerConfig(config, port);
    std::cout << config.toString() << std::endl;
    // get sensor configuration
    //std::cout << "Query sensor config!!" << std::endl;
    SensorConfig sConfig;
    QuerySensorConfig(sConfig, port);    
    //std::cout << sConfig.toString() << std::endl;
    
    // wait for the sensor controller to get ready
    TimerUtil::SleepMs(5000);
    
    // create DSACON with sensor configuration data    
    return new DSACON32(port, config, sConfig);
}

void DSACON32::QueryControllerConfig(ControllerConfig &config, SerialPort &port)
{
    unsigned char data[500];
    createData(QUERY_CONTROLLER_CONFIG,0,data);
    SendBlocking(data,6,QUERY_CONTROLLER_CONFIG,DEFAULT_TIMEOUT,port);
    // wait for controller configuration
    parseControllerCfg(&(data[6]), config);
}

void DSACON32::QuerySensorConfig(SensorConfig &sconfig, SerialPort &port)
{
    unsigned char data[500];
    createData(QUERY_SENSOR_CONFIG,0,data);
    SendBlocking(data,6,QUERY_SENSOR_CONFIG,DEFAULT_TIMEOUT,port);
    parseSensorCfg(&(data[6]), sconfig);
}

void DSACON32::QueryLoopTest(SerialPort &port)
{
    unsigned char data[500];
    createData(LOOP_COMMAND,0,data);
    SendBlocking(data,6,LOOP_COMMAND,DEFAULT_TIMEOUT,port);
}

void DSACON32::QueryControllerState(SerialPort &port)
{
    unsigned char data[6], buffer[500];
    //std::cout << "Query Controller state!!!" << std::endl;
    createData(QUERY_CONTROLLER_STATE,0,data);
    port.write((char*)data, 6);
    ReadAck(buffer, QUERY_CONTROLLER_STATE, DEFAULT_TIMEOUT*2, port);
}

void DSACON32::ReadAck(unsigned char *buffer, unsigned char cmdId, 
						unsigned int timeout, SerialPort &port){
	unsigned char id;
	unsigned short payload;
	long waitUntil = TimerUtil::CurrentTimeMs() + timeout;
	do{
		if( !ReadHeader(buffer, id, payload, timeout, port) )
			RW_THROW("Timeout while recieving header with id" << (int)cmdId);
		
		//if( id != cmdId )
		//	std::cout << "Id: " << (int) id<< " pay: " << payload << std::endl;
		//if( id == 0x0a )
		//	std::cout << "ErrorCode: " << ConvertUtil::ToInt16(buffer, 6) << std::endl;
			//RW_THROW("Wrong packet id recieved! " << (int)id << " wanted " << (int)cmdId);
	} while( id!=cmdId && waitUntil>TimerUtil::CurrentTimeMs());
}

int staticPreambleIdx = 0;

bool DSACON32::ReadHeader(unsigned char *buffer, unsigned char &id, 
						  unsigned short &payload, unsigned int timeout,
						  SerialPort &port){
    // search for preamble
	//int preambleIdx = 0;
	//std::cout << "Preamble: " << staticPreambleIdx << std::endl; 
	do {
		//std::cout << "Preamble: " << preambleIdx << std::endl;
		if( !port.read((char *) buffer, 1, timeout, 10) ){
			//std::cout << "PREAMBLE: timeout" << std::endl;
			return false;
		}
		if( buffer[0]==0xAA ){
			staticPreambleIdx++;
		} else {
			staticPreambleIdx=0;
		}
	}while( staticPreambleIdx<3);
	staticPreambleIdx=0;
	// now read id and payload size of header 3 and [4:5]
    if( !port.read((char *) &(buffer[3]), 3, timeout, 1) ){
    	RW_WARN("Read operation on serialport timed out!");
    	return false;
    }
    // save id
    id = buffer[3];
    // save payload
    payload = ConvertUtil::ToInt16(buffer, 4);
    
	if(payload>0){
    	//std::cout << "Payloadsize: " << payload << std::endl;
        if( !port.read((char*)&(buffer[6]), payload+1, timeout, 20) ){
        	RW_WARN("payloadtimed out!");
            return false;
        }
    }

    return true;
}

bool DSACON32::queryControllerConfig()
{
    unsigned char data[6];
    createData(QUERY_CONTROLLER_CONFIG,0,data);
    _sPort->write((char*)data, 6);
    //return readAck(QUERY_CONTROLLER_CONFIG, DEFAULT_TIMEOUT*2);
    return true;
}

bool DSACON32::querySensorConfig()
{
    unsigned char data[6];
    createData(QUERY_SENSOR_CONFIG,0,data);
    _sPort->write((char*)data, 6);
    //return readAck(QUERY_SENSOR_CONFIG, DEFAULT_TIMEOUT*2);
    return true;
}

bool DSACON32::queryLoopTest()
{
    unsigned char data[6];
    createData(LOOP_COMMAND,0,data);
    _sPort->write((char*)data, 6);
    //return readAck(LOOP_COMMAND, DEFAULT_TIMEOUT*2);
    return true;
}

bool DSACON32::queryControllerState()
{
    unsigned char data[6];
    createData(QUERY_CONTROLLER_STATE,0,data);
    _sPort->write((char*)data, 6);
    //return readAck(QUERY_CONTROLLER_STATE, DEFAULT_TIMEOUT*2);
    return true;
}


bool DSACON32::startDataAcquisition()
{
	if( isDataAcqRunning() )
		return false;
	
	unsigned char data[10];
	createData(START_DATA_ACQ, 3, data);

    // Flags
    if (_useCompression)
        data[6] = 0xA1;   // With compression
    else
        data[6] = 0xA0;   // Without compression

    // Framerate
    data[7] =  _fps & 0xFF;
    data[8] = (_fps>>8)&0xFF ;

    //std::cout << "fpgs: " << (int) data[7] << " ; " << (int)data[8] << std::endl;
    // Checksum
    data[9] = static_cast<unsigned char>(data[6] + data[7] + data[8]);
    
    SendBlocking(data, 10, START_DATA_ACQ, DEFAULT_TIMEOUT, *_sPort);
    QueryControllerConfig(_cConfig, *_sPort);
    //
    return true;
}

bool DSACON32::stopDataAcquisition()
{    
    unsigned char data[10];
    createData(START_DATA_ACQ, 3, data);
    
    // Payload Data
    data[6] = 0x00;
    data[7] = 0x00;
    data[8] = 0x00;

    // Checksum
    data[9] = 0x00;

    _sPort->write((char*)data, 10);
    //bool gotAck = readAck(START_DATA_ACQ, DEFAULT_TIMEOUT*2);
    return true;
}


bool DSACON32::readAck(unsigned char cmdId, unsigned int timeout){
	unsigned char id;
	unsigned short payload;
	
	if( !readHeader(id, payload, timeout) )
		RW_THROW("Timeout while recieving header with id" << (int)cmdId);
	std::cout << "Id: " << (int)id << " payload: " << (int)payload << std::endl;
	if( id != cmdId )
		RW_WARN("Wrong packet id recieved! " << (int)id << " wanted " << (int)cmdId);
	parsePacket(_buffer, id, payload);
	return id==cmdId;
}


bool DSACON32::readThreadFunc()
{
	unsigned char id;
	unsigned short payload;
	//std::cout << "Read Header! " << std::endl;
	if( !readHeader(id,payload,100) ){
		//std::cout << "Timeout!!" << std::endl;
		return false;
	}
	//std::cout << "Id: " << (int)id << " payload: " << (int)payload << std::endl;
    // parse the body 
	//std::cout << "parse packet" << std::endl; 
    parsePacket(_buffer, id, payload);
    return true;
}

void DSACON32::parsePacket(unsigned char data[], unsigned char id, unsigned short payloadSize)
{
	//std::cout << "DSACON32::Parse packet" <<std::endl;
    int errorCode;
    switch (id)
    {
    case QUERY_DATA_PACKET:      // Data frame
    	//std::cout << "DSACON32::Data frame" << std::endl;
        parseDataFrame(data, payloadSize);
        _lastDataAcqTime = TimerUtil::CurrentTimeMs();
        break;
    case QUERY_CONTROLLER_CONFIG:      // Controller config
    	parseControllerCfg(data,_cConfig);
        break;
    case QUERY_SENSOR_CONFIG:      // Sensor config
    	parseSensorCfg(data, _sConfig);
    	break;
    case START_DATA_ACQ:      // Data acquisition
    	//std::cout << "DSACON32::data acquisition" << !_isDataAckRunning << std::endl;
        errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_THROW("Parse error: " << errorCode);
        _isDataAckRunning = !_isDataAckRunning;
        break;
    case LOOP_COMMAND:      // Loop command
        break;
    case QUERY_CONTROLLER_STATE:      // Controller state
        errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_THROW("Parse error: " << errorCode);
        //OnCtrlState(errorCode);
        break;
    default:
    	RW_THROW("BAD ID RECIEVED" << (int) id);
    }
}

void DSACON32::parseDataFrame(unsigned char data[], unsigned int payloadSize)
{
    int time = ConvertUtil::ToInt32(data, 6);
    if (_useCompression)
    {
    	std::cout << "USING COMPRESSION!!!!" << std::endl;
        bool bPadA = true;
        int i = 0;
        int j = 0;
        int numFrameData = payloadSize - 4;
        for (int k = 0; k < numFrameData; k++)
        {
            unsigned char loByte = data[k + 10];
            k++;
            unsigned char hiByte = data[k + 10];
            int count = hiByte >> 4;
            short tmp = (short)(hiByte & 0x0F);
            tmp <<= 8;
            tmp += loByte;
            while (count > 0)
            {
                if (bPadA)
                    _padA.set(i,j,tmp);
                else
                	_padB.set(i,j,tmp);
                i++;
                if (i >= 6)
                {
                    i = 0;
                    j++;
                    if (j >= 14)
                    {
                        j = 0;
                        bPadA = false;
                    }
                }
                count--;
            }
        }
    }
    else
    {
        int k = 10;
        for (int j = 0; j < 14; j++){
            for (int i = 0; i < 6; i++){
            	_padA.set(i,j, ConvertUtil::ToInt16(data, k) );
                k += 2;
            }
        }
        for (int j = 0; j < 14; j++){
            for (int i = 0; i < 6; i++){
            	_padB.set(i,j, ConvertUtil::ToInt16(data, k) );
                k += 2;
            }
        }
    }
    _timeStamp = time;
}


bool DSACON32::readHeader(unsigned char &id, unsigned short &payload, unsigned int timeout){
    // search for preamble
	do {
		//std::cout << "Preamble: " << _preambleIdx << std::endl;
		if( !_sPort->read((char *) _buffer, 1, timeout, 1) ){
			//std::cout << "Nothing to read timeout!!!" << std::endl;
			return false;
		}
		if( _buffer[0]==0xAA ){
			_preambleIdx++;
		} else {
			_preambleIdx=0;
		}
	}while( _preambleIdx<3);
	// remember to reset preamble index
	_preambleIdx=0;
		
	// now read id and payload size of header 3 and [4:5]
    if( !_sPort->read((char *) &(_buffer[3]), 3, timeout, 1) ){
    	//RW_THROW("Read operation on serialport timed out!");
    	return false;
    }
    // save id
    id = _buffer[3];
    // save payload
    payload = ConvertUtil::ToInt16(_buffer, 4);
    
	if(payload>0){
    	//std::cout << "Payloadsize: " << payloadSize << std::endl;
        if( !_sPort->read((char*)&(_buffer[6]), payload+1, timeout, 1) ){
            return false;
        }
    }

    return true;
}

