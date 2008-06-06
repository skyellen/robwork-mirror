#include "ESDCANPort.hpp"

#include <rw/common/TimerUtil.hpp>
#include <rw/common/macros.hpp>

 

using namespace rwlibs::io;

namespace {
        
    
    bool setBaud(HANDLE &handle, ESDCANPort::CanBaud baud){
    	DWORD b = baud;
        long ret = canSetBaudrate( handle, b );
        if(ret != NTCAN_SUCCESS){
            RW_WARN("Cannot set baud to "<< baud << " " << ret);
            return false;
        }
        return true;
    }
    
    void printFifoInfo(HANDLE &handle){
    	long lArg;
    	DWORD status;
    	status = canIoctl( handle, NTCAN_IOCTL_GET_RX_MSG_COUNT, &lArg);
    	std::cout << "Msgs in rx fifo queue: " << lArg << std::endl;
    }

    bool openDevice(unsigned int netId, long txQueueSize, long rxQueueSize, HANDLE &handle){
        unsigned long mode = 0;
        long txtimeout = 100;
        long rxtimeout = 100;
        
        long ret = canOpen(netId, 
        					mode, 
        					txQueueSize, 
        					rxQueueSize, 
        					txtimeout, 
        					rxtimeout, 
        					&handle);
        
        if(ret != NTCAN_SUCCESS){
            //std::cout << "Cannot open Net-Device " << netId << " " << ret << std::endl;
            canClose(handle);
            return false;
        }
        
        return true;
    }
        
     bool getStatus(unsigned int netId, ESDCANPort::CanDeviceStatus& status){
        HANDLE h0;
        CAN_IF_STATUS cstat;
        
        //std::cout << "Tesing if can device exists: " << std::endl;
        
        if( !openDevice(netId, 1, 1, h0) )
            return false;
        
        long ret;
        ret = canStatus(h0, &cstat);
        if(ret != NTCAN_SUCCESS){
            //std::cout << "Cannot get Status of Net-Device "<< netId << std::endl;
            canClose(h0);
            return false;
        }

        uint32_t baudrate;
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

ESDCANPort::ESDCANPort(unsigned int netId, long txQueueSize, long rxQueueSize, CanBaud canBaud, int transmitDelay):
	_netId( netId ),
	_txQueueSize(txQueueSize),
	_rxQueueSize(rxQueueSize),
	_canBaud( canBaud),
	_transmitDelay(transmitDelay),
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
    
    for(unsigned int i=0; i < 255; i++){
        CanDeviceStatus status;
        bool available = getStatus(i, status);
        if( available ){
            canDevices.push_back(status);
        }
    }
    return canDevices;
}


ESDCANPort* ESDCANPort::getPortInstance(unsigned int netId, long txQueueSize, long rxQueueSize, CanBaud canBaud, int transmitDelay) // TODO: add baud ad can id type
{
    CanDeviceStatus status;
    bool available = getStatus(netId, status);
    if( !available )
        return NULL;
    // TODO: add the instance to a static list so that we don't risk instantiating it twice
    return new ESDCANPort(status.netid, txQueueSize, rxQueueSize, canBaud, transmitDelay);
}

bool ESDCANPort::isOpen(){
	return _portOpen;
}

void ESDCANPort::setBaudRate(CanBaud canBaud){
	_canBaud = canBaud;
}

bool ESDCANPort::open(){
	return open(0x00, 0xFF);
}

bool ESDCANPort::open(int idlow, int idhigh){
    _portOpen = false;
    
    if( !openDevice(_netId, _txQueueSize, _rxQueueSize, _handle) ){
        RW_WARN("Port cannot be openned!");
        return false;
    }
    
	/*HANDLE localHandle;

	if( !openDevice(_netId, _txQueueSize, _rxQueueSize, localHandle) ){
        RW_WARN("Port cannot be openned!");
        return false;
    }

	std::cout << "handle is " << _handle << " local handle is " << localHandle << std::endl;
*/
    _portOpen = true;
    
    bool isBaudSet=false; 
    while(!isBaudSet){
        rw::common::TimerUtil::sleepMs(20);
        isBaudSet = setBaud(_handle, _canBaud);
    }
    
    long ret;

	
	//RW_WARN("\n\n\n\n!!! This is a special version if ESDCAN for the LWA that only listens on ID's 163..169 !!!!\n\n\n\n\n");

	//int lwa_ack_can_id[7]={163,164,165,166,167,168,169};
	//int lwa_ack_can_id[7]={123,124,125,126,127,128,129}; //

	for(size_t i=idlow; i<=idhigh; i++){
        ret = canIdAdd(_handle, i);
        if( ret != NTCAN_SUCCESS ){
            std::cout << "Cannot add id's! " << (unsigned short)ret << std::endl; 
            continue;        
        }
    }
    // TODO: Following does not work as it should
    // enable recieving of following ids
    //long fromId = 0, toId = 1024;    
    //ret = canIdRangeAdd(_handle, fromId, toId);  
        
    uint32_t baud;
    ret = canGetBaudrate(_handle, &baud);
    if( ret != NTCAN_SUCCESS ){
        RW_WARN("Cannot read baudrate! " << (unsigned short)ret); 
        return false;        
    }    
	return true;
}

void ESDCANPort::close(){
	_portOpen = false;
	long ret = canClose(_handle);
    if( ret != NTCAN_SUCCESS )
        RW_THROW("Error occured when closing can port!" << (unsigned short)ret);
    
}

bool ESDCANPort::read( CanPort::CanMessage  &msg){
    int32_t nrOfMsg = 1;
    CMSG canMsgBuff[1];
    
    // read one message from rx-fifo using non-blocking call 
    //long ret = canTake(_handle, canMsgBuff, &nrOfMsg);    
    long ret = canRead(_handle, &canMsgBuff[0], &nrOfMsg, NULL);
    
    // check if any error occurred
    if( ret != NTCAN_SUCCESS ){
        //RW_WARN("Error reading port! " << (unsigned short)ret); 
        return false;
    }
    
    // check if any message was read
    
    if( nrOfMsg <1 ){
        return false;
    }
        
    // the message was read convert data to rw can msg type
    //msg.timeStamp = canMsgBuff[0].
    msg.id = canMsgBuff[0].id;
    msg.length = canMsgBuff[0].len;
    for(size_t i=0; i<msg.length; i++){
        msg.data[i] =  canMsgBuff[0].data[i];
    }
	return true;
}

bool ESDCANPort::write(
    unsigned int id, const std::vector<unsigned char>& raw_data)
{
    // Take a copy so that we can provide constness of the input.
    int32_t nrOfMsg = 1;
    CMSG msgBuff;
    msgBuff.id = id;
    msgBuff.len = raw_data.size();
    for(size_t i=0; i<raw_data.size(); i++){
        msgBuff.data[i] = raw_data[i];
    }   
    
    //long ret = canSend(_handle, &msgBuff, &nrOfMsg);
    long ret = canWrite(_handle, &msgBuff, &nrOfMsg, NULL);
    rw::common::TimerUtil::sleepMs(_transmitDelay);
    
    // check if any error occurred
    if( ret != NTCAN_SUCCESS ){
        RW_WARN("Error writing: " << (unsigned short)ret);
        return false;
    }
    
    // check if any message was written
    if( nrOfMsg <1 ){
        RW_WARN("No MSG was written");
        return false;
    }
    
    return true;
}
