
#include "icanapi.h"

#include <iostream>

CAN_CountCardsType CAN_CountCards;
CAN_ReadBoardInfoType CAN_ReadBoardInfo;
CAN_ModeType CAN_Mode;
CAN_InitType CAN_Init;
CAN_ResetType CAN_Reset;
CAN_ReadCanInfoType CAN_ReadCanInfo;
CAN_ReadCanStatusType CAN_ReadCanStatus;
CAN_StartType CAN_Start;
CAN_IsRunType CAN_IsRun;
CAN_ConfigQueueType CAN_ConfigQueue;
CAN_SetAccMaskType CAN_SetAccMask;
CAN_CountMsgsType CAN_CountMsgs;
CAN_SendMsgType CAN_SendMsg;
CAN_ReadMsgType CAN_ReadMsg;
CAN_RequestMsgType CAN_RequestMsg;
CAN_UpdateBufObjType CAN_UpdateBufObj;
CAN_SelfTestType CAN_SelfTest;

HINSTANCE hDll;
bool _isLibOpen = false;

bool openIEICAN02Library() {
	if( _isLibOpen )
		return true;
	
  hDll = LoadLibrary("IEICAN02.dll");
  //  hDll = GetModuleHandle("IEICAN02");

  if (hDll == NULL) {
    std::cout<<"Error Could not load library"<<std::endl;
    return false;
  } 
  
  _isLibOpen = true;
  // else std::cout << "library opened!!" << std::endl;
  
  CAN_ConfigQueue = (CAN_ConfigQueueType)GetProcAddress(hDll, "?CAN_ConfigQueue@@YGGGEEG@Z");
  if (CAN_ConfigQueue == NULL) 
    std::cout<<"Failed to retrieve CAN_ConfigQueue "
	     << GetLastError() <<std::endl;
  // else std::cout<<"Retrieved CAN_ConfigQueue"<<std::endl;
  

  CAN_CountCards = (CAN_CountCardsType)GetProcAddress(hDll, TEXT("?CAN_CountCards@@YGJXZ"));
  if (CAN_CountCards == NULL) 
    std::cout
      <<"Failed to retrieve CAN_CountCards "
      << GetLastError() <<std::endl;
  // else std::cout<<"Retrieved CAN_CountCards"<<std::endl;
  
  CAN_ReadBoardInfo = (CAN_ReadBoardInfoType)GetProcAddress(hDll, "?CAN_ReadBoardInfo@@YGJGPAUCAN_BOARD_INFO@@@Z");
  if (CAN_ReadBoardInfo == NULL) 
    std::cout<<"Failed to retrieve CAN_ReadBoardInfo"<<std::endl;
  // else std::cout<<"Retrieved CAN_ReadBoardInfo"<<std::endl;
  
  CAN_Mode = (CAN_ModeType)GetProcAddress(hDll, "?CAN_Mode@@YGJGE@Z");
  if (CAN_Mode == NULL) 
    std::cout<<"Failed to retrieve CAN_Mode"<<std::endl;
  // else std::cout<<"Retrieved CAN_Mode"<<std::endl;
  
  CAN_Init = (CAN_InitType)GetProcAddress(hDll, "?CAN_Init@@YGJGEEEE@Z");
  if (CAN_Init == NULL) 
    std::cout<<"Failed to retrieve CAN_Init"<<std::endl;
  // else std::cout<<"Retrieved CAN_Init"<<std::endl;
  
  CAN_Reset = (CAN_ResetType)GetProcAddress(hDll, "?CAN_Reset@@YGJGE@Z");
  if (CAN_Reset == NULL) 
    std::cout<<"Failed to retrieve CAN_Reset"<<std::endl;
  // else std::cout<<"Retrieved CAN_Reset"<<std::endl;
  
  CAN_ReadCanInfo = (CAN_ReadCanInfoType)GetProcAddress(hDll, "?CAN_ReadCanInfo@@YGJGEPAUCAN_INFO@@@Z");
  if (CAN_ReadCanInfo == NULL) 
    std::cout<<"Failed to retrieve CAN_ReadCanInfo"<<std::endl;
  // else std::cout<<"Retrieved CAN_ReadCanInfo"<<std::endl;
  
  CAN_ReadCanStatus = (CAN_ReadCanStatusType)GetProcAddress(hDll, "?CAN_ReadCanStatus@@YGJGEPAUCAN_STS@@@Z");
  if (CAN_ReadCanStatus == NULL) 
    std::cout<<"Failed to retrieve CAN_ReadCanStatus"<<std::endl;
  // else std::cout<<"Retrieved CAN_ReadCanStatus"<<std::endl;
  
  CAN_Start = (CAN_StartType)GetProcAddress(hDll, "?CAN_Start@@YGJGE@Z");
  if (CAN_Start == NULL) 
    std::cout<<"Failed to retrieve CAN_Start"<<std::endl;
  // else std::cout<<"Retrieved CAN_Start"<<std::endl;
  
  CAN_IsRun = (CAN_IsRunType)GetProcAddress(hDll, "?CAN_IsRun@@YG_NGE@Z");
  if (CAN_IsRun == NULL) 
		std::cout<<"Failed to retrieve CAN_IsRun"<<std::endl;
  // else std::cout<<"Retrieved CAN_IsRun"<<std::endl;

  CAN_SetAccMask = (CAN_SetAccMaskType)GetProcAddress(hDll, "?CAN_SetAccMask@@YGJGEKK@Z");
  if (CAN_SetAccMask == NULL) 
    std::cout<<"Failed to retrieve CAN_SetAccMask"<<std::endl;
  // else std::cout<<"Retrieved CAN_SetAccMask"<<std::endl;
  
  CAN_CountMsgs = (CAN_CountMsgsType)GetProcAddress(hDll, "?CAN_CountMsgs@@YGJGEE@Z");
  if (CAN_CountMsgs == NULL) 
    std::cout<<"Failed to retrieve CAN_CountMsgs"<<std::endl;
  // else std::cout<<"Retrieved CAN_CountMsgs"<<std::endl;
  
  CAN_SendMsg = (CAN_SendMsgType)GetProcAddress(hDll, "?CAN_SendMsg@@YGJGEKEPAE@Z");
  if (CAN_SendMsg == NULL) 
    std::cout<<"Failed to retrieve CAN_SendMsg"<<std::endl;
  // else std::cout<<"Retrieved CAN_SendMsg"<<std::endl;
  
  CAN_ReadMsg = (CAN_ReadMsgType)GetProcAddress(hDll, "?CAN_ReadMsg@@YGJGEGPAUCAN_MSG@@@Z");
  if (CAN_ReadMsg == NULL) 
    std::cout<<"Failed to retrieve CAN_ReadMsg"<<std::endl;
  // else std::cout<<"Retrieved CAN_ReadMsg"<<std::endl;
  
  CAN_RequestMsg = (CAN_RequestMsgType)GetProcAddress(hDll, "?CAN_RequestMsg@@YGJGEKE@Z");
  if (CAN_RequestMsg == NULL) 
    std::cout<<"Failed to retrieve CAN_RequestMsg"<<std::endl;
  // else std::cout<<"Retrieved CAN_RequestMsg"<<std::endl;
  
  CAN_UpdateBufObj = (CAN_UpdateBufObjType)GetProcAddress(hDll, "?CAN_UpdateBufObj@@YGJGEEPAE@Z");
  if (CAN_UpdateBufObj == NULL) 
    std::cout<<"Failed to retrieve CAN_UpdateBufObj"<<std::endl;
  // else std::cout<<"Retrieved CAN_UpdateBufObj"<<std::endl;
  
  CAN_SelfTest = (CAN_SelfTestType)GetProcAddress(hDll, "?CAN_SelfTest@@YGJGE@Z");
  if (CAN_SelfTest == NULL) 
    std::cout<<"Failed to retrieve CAN_SelfTest"<<std::endl;
  // else std::cout<<"Retrieved CAN_SelfTest"<<std::endl;
  
  return true;
}

bool isIEICAN02LibOpen(){
	return _isLibOpen;	
}

void closeIEICAN02Library() 
{
	if( !_isLibOpen )
		return;
	_isLibOpen = false;
	FreeLibrary(hDll);
}
