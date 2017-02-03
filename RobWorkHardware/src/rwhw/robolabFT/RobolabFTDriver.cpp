/*
 * robolabFT.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: trs
 */


#include "RobolabFTDriver.hpp"

#include <rw/common/TimerUtil.hpp>

#include <iostream>
#include <string>
#include <stdlib.h>     /* strtol */

using namespace rw::common;

using namespace rwhw;

RobolabFT::RobolabFT():
	_isRunning(false),
	_dataInit(false)
{
}
RobolabFT::~RobolabFT(){
	_serialPort.close();
	if (_dataInit) {
		delete[] _dataIn;
		delete[] _dataOut;
	}
}
bool RobolabFT::init(const std::string& port, SerialPort::Baudrate baudrate, int sensors){
	 if(connect(port,baudrate)){
		 if (_dataInit) {
			 delete[] _dataIn;
			 delete[] _dataOut;
		 }
		 _dataIn = new char[7*sensors+1];
		 _dataOut = new char[7*sensors+1];
		 _dataInit = true;
		 _sensors=sensors;
		 _receiveThread = boost::thread(&RobolabFT::runReceive, this);
		 _stopThread=false;

		 return true;
	 }
	 else return false;
}
bool RobolabFT::connect(const std::string& port, SerialPort::Baudrate baudrate){
	if (_serialPort.open(port, baudrate)) {
		std::cout << "connected to " <<port<< std::endl;
		return true;
	}
	RW_WARN("Unable to connect to RobolabFT");
	return false;
}
RobolabFT::RobolabFTData RobolabFT::read(){
	RobolabFT::RobolabFTData tmpData;
	for(int j=0;j<3;j++){
		tmpData.data.first[j]=0;
		tmpData.data.second[j]=0;
	}
	std::string subString;

	int value=0;
	boost::unique_lock<boost::mutex> lock(_mutex);
	tmpData.timestamp= _timestamp;
	std::string str(_dataOut);
	//std::cout<<"str: "<<str<<"_dataOut is :"<<_dataOut<<std::endl;
	int lenght = str.find("\n");
	//if(lenght>0){
		str=str.substr(0,lenght);

		int found = str.find(" ");
		if(lenght==0)std::cout<<"lenght: "<<lenght<<" found: "<<found<<"_dataOut is:"<<_dataOut<<"str is:"<<str<<std::endl;
		subString=str.substr(0,found);
		value= atoi(subString.c_str());
		tmpData.data.first[0] = double (value);
		for(int j=1;j<3;j++){
			if(found<lenght){
				str=str.substr(found+1);
				subString=str.substr(0,found);
				value= atoi(subString.c_str());
				tmpData.data.first[j] = double (value);
				found = str.find(" ");
			}
		}
		for(int j=0;j<3;j++){
			if(found<lenght){
				str=str.substr(found+1);
				subString=str.substr(0,found);
				value= atoi(subString.c_str());
				tmpData.data.second[j] = double (value);
				found = str.find(" ");
			}
		}
	//}
	//	boost::unique_lock<boost::mutex> unlock(_mutex);
	std::cout<<"foces: "<<tmpData.data.first<<"tourques: "<<tmpData.data.second<<std::endl;
	return tmpData;
}
bool RobolabFT::updateData(){
	/*if(n>0){
		//_dataIn.find("\n");
		std::cout<<"n is "<< n <<": "<< _dataIn << std::endl;
		return true;
	}*/
	return false;
}

bool RobolabFT::write(){
	return false;
}
void RobolabFT::run(){
	_isRunning=true;
	do{

		read();
		TimerUtil::sleepMs(1000);
	}while(_isRunning);

}
void RobolabFT::stop(){
	if(_threadRunning) {
	       _stopThread = true;

	       // Give the thread one second to stop
	       if(!_receiveThread.timed_join(boost::posix_time::seconds(1))) {
	           // Failure, interrupt
	           RW_WARN("Interrupting receive thread...");
	           _receiveThread.interrupt();
	           if(!_receiveThread.timed_join(boost::posix_time::seconds(1)))
	               RW_WARN("Failed to interrupt receive thread");
	       }
	       else{
	    	   RW_LOG_INFO("Thread stopped...");
	       }
	   }

	 _serialPort.close();



}

//thread function
void RobolabFT::runReceive(){
	 std::cout<<"starting receive thread" << std::endl;
	 const unsigned int timeout = 200;
	 bool readOK=false;
	try{
		int n=_sensors*7 + 1;
		_threadRunning = true;
	    //unsigned char data[70];
		 while(!_stopThread) {

			 while(_dataIn[0]!='\n'){ //Find star of the tranfer
				 readOK =_serialPort.read((char*)_dataIn, 1,timeout,1);

			 }
			 readOK =_serialPort.read((char*)_dataIn, n,timeout,1);
			 if(readOK){

				 boost::unique_lock<boost::mutex> lock(_mutex);
				 _timestamp = TimerUtil::currentTime();
				 //std::cout<<"_dataOut= ";
				 for(int i=0;i<n;i++){
					 _dataOut[i]=_dataIn[i];
					// std::cout<<_dataOut[i];
				 }
				 //boost::unique_lock<boost::mutex> unlock(_mutex);
				 //std::cout<< "Out is now :"<<_dataOut<<std::endl;
			 }
		}
	}
	catch(const std::exception& e) {
        _threadRunning = false;
        RW_WARN("Exception caught during reception: " << e.what());
	}
}
