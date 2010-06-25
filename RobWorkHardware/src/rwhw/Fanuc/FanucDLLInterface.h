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

#ifndef RWHW_FANUC_DLLINTERFACE_H_
#define RWHW_FANUC_DLLINTERFACE_H_

#include <windows.h>
#include <iostream>

typedef void* InterfaceHandle;
typedef void* TcpRegisterHandle;
typedef void* JointRegisterHandle;
typedef void* RegisterHandle;

/**
 * @brief struct for communicating poses with the Fanuc DLL.
 */
typedef struct _Txyzwpr {
    float X;
    float Y;
    float Z;
    float W;
    float P;
    float R;
} Txyzwpr;

/**
 * @brief struct for communicating joint configurations with the Fanuc DLL.
 */
typedef struct _TJoints {
    float q[6];
} TJoints;

void print(_TJoints q){
  std::cout << "{"<<q.q[0]<<","<<q.q[1]<<","
      <<q.q[2]<<","<<q.q[3]<<","<<q.q[4]
      <<","<<q.q[5]<<"}" << std::endl;
}

typedef void* (__stdcall *InitInterfaceType)(char *ipNr, int updateRate);
InitInterfaceType InitInterface;

typedef bool (__stdcall *CloseInterfaceType)(InterfaceHandle handle);
CloseInterfaceType CloseInterface;

typedef TcpRegisterHandle (__stdcall *AddTcpRegisterType)(InterfaceHandle handle, int regNr);
AddTcpRegisterType AddTcpRegister;

typedef bool (__stdcall * WriteTcpRegType)(TcpRegisterHandle reg, Txyzwpr *tcpValue);
WriteTcpRegType WriteTcpReg;

typedef bool (__stdcall * WriteJointRegType)(JointRegisterHandle reg, TJoints *qValue);
WriteJointRegType WriteJointReg;

typedef bool (__stdcall * ReadTcpRegType)(TcpRegisterHandle reg, Txyzwpr *tcpValue);
ReadTcpRegType ReadTcpReg;

typedef JointRegisterHandle (__stdcall * AddJointRegisterType)(InterfaceHandle handle, int regNr);
AddJointRegisterType AddJointRegister;

typedef bool (__stdcall * ReadJointRegType)(JointRegisterHandle reg, TJoints *qValue);
ReadJointRegType ReadJointReg;

typedef RegisterHandle (__stdcall * AddRegisterType)(InterfaceHandle handle, int regNr);
AddRegisterType AddRegister;

typedef bool (__stdcall * WriteRegisterType)(RegisterHandle handle, float value);
WriteRegisterType WriteRegister;

typedef bool (__stdcall * ReadRegisterType)(RegisterHandle handle, float *value);
ReadRegisterType ReadRegister;

typedef bool (__stdcall * WriteSpeedType)(InterfaceHandle handle, int value);
WriteSpeedType WriteSpeed;

typedef int (__stdcall * ReadSpeedType)(InterfaceHandle handle);
ReadSpeedType ReadSpeed;

typedef bool (__stdcall * ReadTcpType)(InterfaceHandle, Txyzwpr *value);
ReadTcpType ReadTcp;

typedef bool (__stdcall * ReadJointsType)(InterfaceHandle, TJoints *value);
ReadJointsType ReadJoints;

typedef char* (__stdcall * VersionInfoType)();
VersionInfoType VersionInfo;

HINSTANCE GibFanucComHandle;

bool openLibrary() {
    GibFanucComHandle = LoadLibrary("GibFanucCom.dll");
    if (GibFanucComHandle == NULL) {
        std::cout<<"Error Could not load library"<<std::endl;
        return false;
    }

    InitInterface = (InitInterfaceType)GetProcAddress(GibFanucComHandle, "InitInterface");
    if (InitInterface == NULL)
        std::cerr<<"Failed to retrieve InitInterface"<<std::endl;
//	else
//		std::cerr<<"Retrieved InitInterface"<<std::endl;

    CloseInterface = (CloseInterfaceType)GetProcAddress(GibFanucComHandle, "CloseInterface");
    if (CloseInterface == NULL)
        std::cerr<<"Failed to retrieve CloseInterface"<<std::endl;
//	else
//		std::cerr<<"Retrieved CloseInterface"<<std::endl;

    AddTcpRegister = (AddTcpRegisterType)GetProcAddress(GibFanucComHandle, "AddTcpRegister");
    if (AddTcpRegister == NULL)
        std::cerr<<"Failed to retrieve AddTcpRegister"<<std::endl;
//	else
//		std::cerr<<"Retrieved AddTcpRegister"<<std::endl;

    WriteTcpReg = (WriteTcpRegType)GetProcAddress(GibFanucComHandle, "WriteTcpReg");
    if (WriteTcpReg == NULL)
        std::cerr<<"Failed to retrieve WriteTcpReg"<<std::endl;
//	else
//		std::cerr<<"Retrieved WriteTcpReg"<<std::endl;

    ReadTcpReg = (ReadTcpRegType)GetProcAddress(GibFanucComHandle, "ReadTcpReg");
    if (ReadTcpReg == NULL)
        std::cerr<<"Failed to retrieve ReadTcpReg"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadTcpReg"<<std::endl;

    AddJointRegister = (AddJointRegisterType)GetProcAddress(GibFanucComHandle, "AddJointRegister");
    if (AddJointRegister == NULL)
        std::cerr<<"Failed to retrieve AddJointRegister"<<std::endl;
//	else
//		std::cerr<<"Retrieved AddJointRegister"<<std::endl;

    WriteJointReg = (WriteJointRegType)GetProcAddress(GibFanucComHandle, "WriteJointReg");
    if (WriteJointReg == NULL)
        std::cerr<<"Failed to retrieve WriteJointReg"<<std::endl;
//	else
//		std::cerr<<"Retrieved WriteJointReg"<<std::endl;

    ReadJointReg = (ReadJointRegType)GetProcAddress(GibFanucComHandle, "ReadJointReg");
    if (ReadJointReg == NULL)
        std::cerr<<"Failed to retrieve ReadJointReg"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadJointReg"<<std::endl;

    AddRegister = (AddRegisterType)GetProcAddress(GibFanucComHandle, "AddRegister");
    if (AddRegister == NULL)
        std::cerr<<"Failed to retrieve AddRegister"<<std::endl;
//	else
//		std::cerr<<"Retrieved AddRegister"<<std::endl;

    WriteRegister = (WriteRegisterType)GetProcAddress(GibFanucComHandle, "WriteRegister");
    if (WriteRegister == NULL)
        std::cerr<<"Failed to retrieve WriteRegister"<<std::endl;
//	else
//		std::cerr<<"Retrieved WriteRegister"<<std::endl;

    ReadRegister = (ReadRegisterType)GetProcAddress(GibFanucComHandle, "ReadRegister");
    if (ReadRegister == NULL)
        std::cerr<<"Failed to retrieve ReadRegister"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadRegister"<<std::endl;

    WriteSpeed = (WriteSpeedType)GetProcAddress(GibFanucComHandle, "WriteSpeed");
    if (WriteSpeed == NULL)
        std::cerr<<"Failed to retrieve WriteSpeed"<<std::endl;
//	else
//		std::cerr<<"Retrieved WriteSpeed"<<std::endl;

    ReadSpeed = (ReadSpeedType)GetProcAddress(GibFanucComHandle, "ReadSpeed");
    if (ReadSpeed == NULL)
        std::cerr<<"Failed to retrieve ReadSpeed"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadSpeed"<<std::endl;

    ReadTcp = (ReadTcpType)GetProcAddress(GibFanucComHandle, "ReadTcp");
    if (ReadTcp == NULL)
        std::cerr<<"Failed to retrieve ReadTcp"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadTcp"<<std::endl;

    ReadJoints = (ReadJointsType)GetProcAddress(GibFanucComHandle, "ReadJoints");
    if (ReadJoints == NULL)
        std::cerr<<"Failed to retrieve ReadJoints"<<std::endl;
//	else
//		std::cerr<<"Retrieved ReadJoints"<<std::endl;

    VersionInfo = (VersionInfoType)GetProcAddress(GibFanucComHandle, "VersionInfo");
    if (VersionInfo == NULL)
        std::cerr<<"Failed to retrieve VersionInfo"<<std::endl;
//	else
//		std::cerr<<"Retrieved VersionInfo"<<std::endl;

    return true;
}

void closeLibrary() {
    FreeLibrary(GibFanucComHandle);
}


#endif /*RWHW_FANUC_DLLINTERFACE_H_*/
