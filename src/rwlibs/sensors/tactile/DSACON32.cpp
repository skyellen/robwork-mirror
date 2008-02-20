#include "DSACON32.hpp"

#include <rw/common/TimerUtil.hpp>
#include <rw/common/ConvertUtil.hpp>
#include <rw/common/macros.hpp>

using namespace rw::common;
using namespace rwlibs::io;
using namespace rwlibs::sensors;

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
   
    
    
}

DSACON32::DSACON32(SerialPort &port, std::pair<int,int> dim):
    _sPort(&port),
    _padA( dim.first , dim.second/2 ),
    _padB( dim.first , dim.second/2 ),
    _dim( dim ),
    _useCompression( false ),
    _fps(30),
    _isDataAckRunning(false)
{
    
}

DSACON32::~DSACON32()
{
}

DSACON32* DSACON32::GetInstance(SerialPort& port) {
    // detect if any DSACON32 unit is connected to this serialPort
    unsigned char data[10];
    createData(0x06, 0, data);
    port.write((char*) data, 6);
    TimerUtil::SleepMs(100);
    
    // read a 0x06, 0
    if( !port.read((char *) data, 6, 100, 1) ){
    	RW_WARN("DSACON32::Loop test failed, no response from DSACON!");
    	return NULL;
    }
    if( data[3] != 0x06 ){
    	RW_WARN("DSACON32::Loop test failed, wrong packet returned! " << (unsigned int)data[3]);
    	return NULL;    	
    }
    
    // get configuration
    
    
    // get sensor configuration
    
    
    // create DSACON with sensor configuration data
    std::pair<int,int> pair(6,28);

    return new DSACON32(port, pair);;
}

void DSACON32::StartDataAcquisition()
{
    if(_isDataAckRunning){
        RW_WARN("DSACON32 data acquisition already running!");
        return;
    }

    unsigned char data[10];
    createData(0x03, 0x03, data);

    // Flags
    if (_useCompression)
        data[6] = 0x81;   // With compression
    else
        data[6] = 0x80;   // Without compression

    // Framerate
    data[7] =  _fps & 0xFF;
    data[8] = (_fps>>8)&0xFF ;

    // Checksum
    data[9] = static_cast<unsigned char>(data[6] + data[7] + data[8]);

    _sPort->write((char*)data, 10);
}

void DSACON32::StopDataAcquisition()
{
    if(!_isDataAckRunning){
        RW_WARN("DSACON32 data acquisition already stopped!");
        return;
    }
    
    unsigned char data[10];
    createData(0x03, 3, data);
    
    // Payload Data
    data[6] = 0x00;
    data[7] = 0x00;
    data[8] = 0x00;

    // Checksum
    data[9] = 0x00;

    _sPort->write((char*)data, 10);
}

void DSACON32::QueryControllerConfig()
{
    unsigned char data[6];
    createData(0x01,0,data);
    _sPort->write((char*)data, 6);
}

void DSACON32::QuerySensorConfig()
{
    unsigned char data[6];
    createData(0x02,0,data);
    _sPort->write((char*)data, 6);
}

void DSACON32::QueryControllerState()
{
    unsigned char data[6];
    createData(0x0A,0,data);
    _sPort->write((char*)data, 6);
}

void DSACON32::ReadThreadFunc()
{
    // search for preamble
	do {
		if( !_sPort->read((char *) _buffer, 1, 100, 1) )
			return;
		
		if( _buffer[0]==0xAA ){
			_preambleIdx++;
		} else {
			_preambleIdx=0;
		}
	}while( _preambleIdx<3);
	// remember to reset preamble index
	_preambleIdx=0;
		
	// now read id and payload size of header 3 and [4:5]
    if( !_sPort->read((char *) &(_buffer[3]), 3, 1000, 1) ){
    	RW_WARN("Read operation on serialport timed out!");
    	return;
    }
    // verify packet id, must be 0x00
    if( _buffer[3]!=0x00 ){
    	return;
    }
    
    // Read payload
    unsigned int payloadSize = ConvertUtil::ToInt16(_buffer, 4);
    if(payloadSize>0){
    	//std::cout << "Payloadsize: " << payloadSize << std::endl;
        if( !_sPort->read((char*)&(_buffer[6]), payloadSize+1, 2000, 1) )
            return;
    }
    // parse the body 
    ParsePacket(_buffer, payloadSize);
}

void DSACON32::ParsePacket(unsigned char data[], unsigned int payloadSize)
{
	//std::cout << "DSACON32::Parse packet" <<std::endl;
    int errorCode;
    unsigned char packet_id = data[3];
    switch (packet_id)
    {
    case 0x00:      // Data frame
    	//std::cout << "DSACON32::Data frame" << std::endl;
        ParseDataFrame(data, payloadSize);
        break;
    case 0x01:      // Controller config
    	//std::cout << "DSACON32::Controller config" << std::endl;
        errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_WARN("Parse error: " << errorCode);
        _cConfig.parse(data);
        std::cout << _cConfig.toString() << std::endl;
        validateChecksum(data, 25);
        break;
    case 0x02:      // Sensor config
    	//std::cout << "DSACON32::Sensor config" << std::endl;
        errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_WARN("Parse error: " << errorCode);
        _sConfig.parse(data);
        std::cout << _sConfig.toString() << std::endl;
        validateChecksum(data, 27);
        break;
    case 0x03:      // Data acquisition
    	//std::cout << "DSACON32::data acquisition" << std::endl;
        errorCode = ConvertUtil::ToInt16(data, 6);
        if (errorCode != 0)
            RW_WARN("Parse error: " << errorCode);
        _isDataAckRunning = !_isDataAckRunning;
        break;
    case 0x06:      // Loop command
    	//std::cout << "DSACON32::Loop" << std::endl;
        break;
    case 0x0A:      // Controller state
        errorCode = ConvertUtil::ToInt16(data, 6);
        //std::cout << "DSACON32::Controller state recieved!"<< std::endl;
        //OnCtrlState(errorCode);
        break;
    }
}

void DSACON32::ParseDataFrame(unsigned char data[], unsigned int payloadSize)
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

void DSACON32::validateChecksum(unsigned char data[], int checksumIndx)
{
    unsigned char checksum = 0;
    for (int i = 6; i < checksumIndx; i++)
        checksum += data[i];
    if (checksum != data[checksumIndx])
        RW_THROW("DSACON32::ValidateChecksum - Checksum error");
}


