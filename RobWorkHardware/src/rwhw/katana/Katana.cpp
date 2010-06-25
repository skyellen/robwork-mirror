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

#include "Katana.hpp"

#include <rw/math/Constants.hpp>

#include <KNI/cdlCOM.h>		//for serial device: TCdlCOMDesc, CCdlCOM
#include <KNI/cplSerial.h>	//for serial-CRC protocol: CCplSerialCRC
#include <KNI/kmlBase.h>	//for robot: CKatBase, CMotBase, CSctBase
#include <KNI/kmlExt.h>		//extenden katana features


using namespace rw::math;
using namespace rwhw;

Katana::Katana(const std::string& configfile):
    _configfile(configfile),
    _communication(NULL),
    _protocol(NULL),
    _katana(NULL)
{

}

Katana::~Katana() {
    if (_katana != NULL)
        disconnectDevice();
}

bool Katana::connectDevice(int port) {
	TCdlCOMDesc ccd = {port, 57600, 8, 'N', 1, 500, 0};	//COM description
    _communication = new CCdlCOM(ccd);					//class creation &
	if (_communication->lastOP() != lopDONE) {					//opens the serial port!
        RW_WARN("Error Opening Communication");
        disconnectDevice();
        return false;
    }
    _protocol = new CCplSerialCRC();		//class creation
	if (!_protocol->init(_communication)) {						//protocol initiation
        RW_WARN("Error Initializing Protocol");
        disconnectDevice();
        return false;
    }
	_katana = new CKatana();					//create katana class
	if (!_katana->create(_configfile.c_str(), _protocol)) {			//init katana
        RW_WARN("Error Initializing Katana");
        disconnectDevice();
        return false;
    }
    return true;

}

void Katana::disconnectDevice() {
    if (_katana != NULL) {
        delete _katana;
        _katana = NULL;
    }
    if (_protocol != NULL) {
        delete _protocol;
        _protocol = NULL;
    }
    if (_communication != NULL) {
        delete _communication;
        _communication = NULL;
    }
}

bool Katana::isConnected() {
    return _katana != NULL;
}

bool Katana::calibrate() {
    if (_katana != NULL) {
        return _katana->calibrate(_configfile.c_str());
    }
    return false;
}

bool Katana::move(const Q& q, bool wait) {
    std::cout<<"Katana::move to "<<q<<std::endl;

    for (int i = 0; i<5; i++) {
        if (_katana->checkENLD(i, q(i)*Rad2Deg)) {
            TRetMOV res = _katana->movDegrees(i, q(i)*Rad2Deg);

            if (res != MOV_TARGET_SET && res != MOV_POSITION_REACHED) {
                std::cout<<"Error in end"<<std::endl;
                return false;
            }
        }
        else
            return false;
    }

    _qcurrent = q;
    return true;
}



Q Katana::getQ() {
    Q q(5);
    CKatBase* base = _katana->GetBase();
    for(int i=0; i<5; i++){
        TRetRecvP res = base->GetMOT()->arr[i].recvPVP();
        if (res != RECV_NO_ERR) {
            RW_THROW("Could not receiver encoder information");
        }
        q(i) = Deg2Rad*(base->GetMOT()->arr[i].GetPVP()->pos * _katana->GetMAP(i).enc_to_deg);
    }
    return q;
}


bool Katana::grip(double pos) {
    _katana->checkENLD(5, pos*Rad2Deg);
    TRetMOV res = _katana->movDegrees(5, pos*Rad2Deg);
    if (res != MOV_TARGET_SET && res != MOV_POSITION_REACHED) {
        std::cout<<"Error in end"<<std::endl;
        return false;
    }
    return true;
}


double Katana::getGripperPos() {
    double pos;
    CKatBase* base = _katana->GetBase();
    for(int i=0; i<5; i++){
        if (base->GetMOT()->arr[i].recvPVP() != RECV_NO_ERR) {
            RW_THROW("Could not receive gripper encoder information");
        }
        pos = base->GetMOT()->arr[i].GetPVP()->pos * _katana->GetMAP(i).enc_to_deg;
    }
    return pos;
}
