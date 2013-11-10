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
#include <string>
#include <rw/rw.hpp>
#include "FalconInterface.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace rw;
using namespace rwhw;



int main(int argc, char* argv[])
{
	FalconInterface falcon;
	
	Vector3D<> neutralPos(0.0, 0.0, 0.04);
	
	falcon.start();
	while (true) {
		string btns;
		unsigned int btnstate = falcon.getButtonState();
		if (btnstate & FalconInterface::ButtonUp) btns += 'W';
		if (btnstate & FalconInterface::ButtonDown) btns += 'S';
		if (btnstate & FalconInterface::ButtonLeft) btns += 'A';
		if (btnstate & FalconInterface::ButtonRight) btns += 'D';
		
		Vector3D<> pos = falcon.getPosition();
		Vector3D<> force = neutralPos - pos;
		falcon.setForce(150*force);
		
		cout << btns << "\t" << pos << endl;
	}
	falcon.stop();
	
	return 0;
}
