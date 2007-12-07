#include "ESDCANPort.hpp"

#include <rw/common/TimerUtil.hpp>

using namespace rwlibs::io;

namespace {
        
    
    bool setBaud(ESDCANPort::CanBaud baud, HANDLE &handle){
        long ret = canSetBaudrate( handle, baud );
        if(ret != NTCAN_SUCCESS){
            std::cout << "Cannot set baud to "<< baud << " " << ret << std::endl;
            return false;
        }
        std::cout << "BAUDRATE successfully set" << std::endl;
        return true;
    }

    bool openDevice(unsigned int netId, HANDLE &handle){
        
        long ret = canOpen(netId, 0, 8, 8, 20, 100, &handle);
        
        if(ret != NTCAN_SUCCESS){
            //std::cout << "Cannot open Net-Device " << netId << " " << ret << std::endl;
            
            canClose(&handle);
            return false;
        }
        
        return true;
    }
        
     bool getStatus(unsigned int netId, ESDCANPort::CanDeviceStatus& status){
        HANDLE h0;
        CAN_IF_STATUS cstat;
        
        //std::cout << "Tesing if can device exists: " << std::endl;
        
        if( !openDevice(netId, h0) )
            return false;
        
        long ret;
        ret = canStatus(h0, &cstat);
        if(ret != NTCAN_SUCCESS){
            //std::cout << "Cannot get Status of Net-Device "<< netId << std::endl;
            canClose(h0);
            return false;
        }

        unsigned long baudrate;
        ret = canGetBaudrate(h0, &baudrate);
        if(ret != NTCAN_SUCCESS){
            //std::cout << "Cannot get Baudrate of Net-Device " << netId << std::endl;
            canClose(h0);
            return false;
        }
        
        printf("Net %3d: ID=%s "
               "Dll=%1X.%1X.%02X "
               "Driver=%1X.%1X.%02X"
               " Firmware=%1X.%1X.%02X\n"
               "         Hardware=%1X.%1X.%02X "
               "Baudrate=%08lx Status=%08lx Features=%04x\n",
               netId, cstat.boardid, 
               cstat.dll     >>12, (cstat.dll     >>8) & 0xf,cstat.dll      & 0xff,  
               cstat.driver  >>12, (cstat.driver  >>8) & 0xf,cstat.driver   & 0xff,  
               cstat.firmware>>12, (cstat.firmware>>8) & 0xf,cstat.firmware & 0xff,  
               cstat.hardware>>12, (cstat.hardware>>8) & 0xf,cstat.hardware & 0xff, 
               (unsigned long)baudrate, cstat.boardstatus, cstat.features );         
        
        status.netid = netId;

        canClose(h0);
        return true;
    }
    
}

ESDCANPort::ESDCANPort(unsigned int netId):
	_netId( netId ),
	_portOpen(false)
{
	
}

ESDCANPort::~ESDCANPort()
{
	if( _portOpen )
		close();
	// if( _instanceCnt == 0 )
	// 		closeIEILibrary()
}

std::vector<ESDCANPort::CanDeviceStatus> ESDCANPort::getConnectedDevices(){
    std::vector<CanDeviceStatus> canDevices;

    std::cout << "Available CAN-Devices:" << std::endl;
    
    for(unsigned int i=0; i < 255; i++){
        CanDeviceStatus status;
        bool available = getStatus(i, status);
        if( available ){
            canDevices.push_back(status);
        }
    }
    return canDevices;
}


ESDCANPort* ESDCANPort::getPortInstance(unsigned int netId) // TODO: add baud ad can id type
{  
    CanDeviceStatus status;
    bool available = getStatus(netId, status);
    if( !available )
        return NULL;
    // TODO: add the instance to a static list so that we don't risk instantiating it twice
    return new ESDCANPort(status.netid);
}

bool ESDCANPort::isOpen(){
	return _portOpen;
}

bool ESDCANPort::open(/* baudrate, 11/29bit option,  */){
    std::cout << "Open " << std::endl;
    _portOpen = false;
    
    if( !openDevice(_netId, _handle) ){
        std::cout << "Port cannot be openned!" << std::endl;
        return false;
    }
    
    _portOpen = true;
    
    bool isBaudSet=false; 
    while(!isBaudSet){
        rw::common::TimerUtil::SleepMs(20);
        isBaudSet = setBaud(CanBaud250, _handle);
    }
    
    // enable recieving of following ids
    long ret = canIdRangeAdd(_handle, 0, 24);
    if( ret != NTCAN_SUCCESS ){
        std::cout << "Cannot add id's! " << (unsigned short)ret << std::endl; 
        return false;        
    }
    
    std::cout << "Success " << std::endl;
	return true;
}

void ESDCANPort::close(){
    std::cout << "Closing port!" << std::endl;
	_portOpen = false;
	canClose(_handle);
}

bool ESDCANPort::read( CanPort::CanMessage  &msg){
    std::cout << "Read: " << std::endl;
    long nrOfMsg = 1;
    CMSG canMsgBuff[1];
    rw::common::TimerUtil::SleepMs(400);
    // read one message from rx-fifo using non-blocking call 
    long ret = canTake(_handle, canMsgBuff, &nrOfMsg);
    // check if any error occurred
    if( ret != NTCAN_SUCCESS ){
        std::cout << "Error reading port! " << (unsigned short)ret << std::endl; 
        return false;
    }
    
    // check if any message was read
    if( nrOfMsg <1 ){
        std::cout << "No msg in rx fifo!" << std::endl;
        return false;
    }
        
    // the message was read convert data to rw can msg type
    //msg.timeStamp = canMsgBuff[0].
    msg.id = canMsgBuff[0].id;
    msg.length = canMsgBuff[0].len;
    for(size_t i=0; i<msg.length; i++){
        msg.data[i] =  canMsgBuff[0].data[i];
    }
    std::cout << "Success " << std::endl;
	return true;
}

bool ESDCANPort::write(
    unsigned int id, const std::vector<unsigned char>& raw_data)
{
    std::cout << "Write: " << std::endl;
    // Take a copy so that we can provide constness of the input.
    long nrOfMsg = 1;
    CMSG msgBuff;
    msgBuff.id = id;
    msgBuff.len = raw_data.size();
    std::cout << "[" << id << "]{" ;
    for(size_t i=0; i<raw_data.size(); i++){
        msgBuff.data[i] = raw_data[i];
        std::cout << (unsigned short)raw_data[i] << ",";
    }
    std::cout << "]" << std::endl;
    
    
    //long ret = canSend(_handle, &msgBuff, &nrOfMsg);
    long ret = canWrite(_handle, &msgBuff, &nrOfMsg, NULL);
    rw::common::TimerUtil::SleepMs(400);
    
    // check if any error occurred
    if( ret != NTCAN_SUCCESS ){
        std::cout << "Error writing: " << (unsigned short)ret << std::endl;
        return false;
    }
    
    // check if any message was written
    if( nrOfMsg <1 ){
        std::cout << "No MSG was written" << std::endl;
        return false;
    }
    
    //
    std::cout << "Success " << std::endl;
    return true;
}
