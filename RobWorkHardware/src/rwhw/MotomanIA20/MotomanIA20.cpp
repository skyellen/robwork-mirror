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

#include <iostream>

#include	<stdio.h>
#include "MotomanIA20.hpp"

using namespace rwhw;

MotomanIA20::MotomanIA20(std::string port)
{
	std::cout << "IA20::IA20()" << std::endl;

	_serialPort = new SerialPort();


	if ( _serialPort == NULL )
		RW_THROW("Serial Port is NULL");

    if ( _serialPort->open( port, SerialPort::Baud9600 ) ){
        std::cout << "\t Serial port " <<  port << " opened" << std::endl;
    } else {
         std::cout << "\t Failed to open serial port " << port << std::endl;
    }
}

MotomanIA20::~MotomanIA20()
{
	std::cout << "IA20::~IA20()" << std::endl;

	_serialPort -> close();

}

void MotomanIA20::setPosition(rw::math::Q q){

	std::vector<int> data;

	// Convert radians to raw encoder values
	// Warning: these factors are probably not very precise!!
	data.push_back( (int)(  104960.77511 * q(0)) );
	data.push_back( (int)( -104960.77511 * q(1)) );
	data.push_back( (int)(  104960.77511 * q(2)) );
	data.push_back( (int)(  104960.77511 * q(3)) );
	data.push_back( (int)(   65843.67320 * q(4)) );
	data.push_back( (int)(  -65843.67320 * q(5)) );
	data.push_back( (int)(   33246.83099 * q(6)) );

	sendRaw(data);

	return;

}

void MotomanIA20::sendRaw(std::vector<int> j_raw_val){

	char buffer[200];
	char bucket[1];


	// Wait for the NX100 controller to signal, with a ',' (comma), that it is ready for the next joint vector
	while ( 1 ) {
		bucket[0] = 0;
		if( _serialPort->read(bucket,1) != 0 )
			std::cout << bucket[0] << std::endl;
		if (bucket[0] == ',')
			break;
		rw::common::TimerUtil::sleepMs(1);
	}


	// The NX100 uses newline (NL) '\n' to detect end of a string.
	sprintf(buffer,"%i,%i,%i,%i,%i,%i,%i\n",
		j_raw_val[0],
		j_raw_val[1],
		j_raw_val[2],
		j_raw_val[3],
		j_raw_val[4],
		j_raw_val[5],
		j_raw_val[6]);

	//std::cout << "IA20::sendRaw() got:\n\t";
	std::cout << buffer;
	std::cout << "\tlength " << strlen(buffer) << std::endl;

	_serialPort->write( buffer , strlen(buffer) );

	return;

}


/*
int main(int argc, char** argv){

	std::cout << "Hello World\n";

	IA20 rob(1);

	IA20 *robother = new IA20(2);

	delete robother;

	return(1);
}
*/
