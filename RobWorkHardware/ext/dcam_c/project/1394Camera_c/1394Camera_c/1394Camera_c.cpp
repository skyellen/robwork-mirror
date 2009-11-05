#include "1394Camera_c.h"

#include <1394Camera.h>

C1394Camera* myCameraList[20];

struct InitStruct {
	InitStruct(){
		for(int i=0;i<20;i++)
			myCameraList[i] = NULL;
	}
};

InitStruct initStruct;

CameraID createCameraInstance_c(){
	// look for the first available cameraid in the list
	for(int i=0;i<20;i++){
		if(myCameraList[i]==NULL){
			myCameraList[i] = new C1394Camera();
			return i;
		}
	}
	return -1;
}

void releaseCameraInstance_c(CameraID id){
	delete myCameraList[id];
	myCameraList[id] = NULL;
}

// Selection/Control
int	RefreshCameraList_c(CameraID id){
	return myCameraList[id]->RefreshCameraList();
}

int	InitCamera_c(CameraID id, BOOL	reset){
	return myCameraList[id]->InitCamera(reset);
}

int	GetNode_c(CameraID id){
	return myCameraList[id]->GetNode();
}

int	GetNodeDescription_c(CameraID id, int node, char *buf,	int	buflen){
	return myCameraList[id]->GetNodeDescription(node,buf,buflen);
}

int	SelectCamera_c(CameraID id, int node){
	return myCameraList[id]->SelectCamera(node);
}

unsigned long GetVersion_c(CameraID id){
	return myCameraList[id]->GetVersion();
}

int	GetNumberCameras_c(CameraID id){
	return myCameraList[id]->GetNumberCameras();
}

void GetCameraName_c(CameraID id,char	*buf, int len){
	myCameraList[id]->GetCameraName(buf,len);
}
void GetCameraVendor_c(CameraID id,char *buf,	int	len){
	myCameraList[id]->GetCameraVendor(buf,len);
}
void GetCameraUniqueID_c(CameraID id,PLARGE_INTEGER pUniqueID){
	myCameraList[id]->GetCameraUniqueID(pUniqueID);
}
int	 GetMaxSpeed_c(CameraID id){
	return myCameraList[id]->GetMaxSpeed();
}
int	 CheckLink_c(CameraID id){
	return myCameraList[id]->CheckLink();
}
bool HasPowerControl_c(CameraID id){
	return myCameraList[id]->HasPowerControl();
}
bool StatusPowerControl_c(CameraID id){
	return myCameraList[id]->StatusPowerControl();
}
int	 SetPowerControl_c(CameraID id,BOOL	on){
	return myCameraList[id]->SetPowerControl(on);
}
bool Has1394b_c(CameraID id){
	return myCameraList[id]->Has1394b();
}
bool Status1394b_c(CameraID id){
	return myCameraList[id]->Status1394b();
}
int	 Set1394b_c(CameraID id,BOOL on){
	return myCameraList[id]->Set1394b(on);
}

// Store/Retrieve Settings from	camera EEPROM
int	MemGetNumChannels_c(CameraID id){
	return myCameraList[id]->MemGetNumChannels();
}
int	MemGetCurrentChannel_c(CameraID id){
	return myCameraList[id]->MemGetCurrentChannel();
}
int	MemLoadChannel_c(CameraID id,int channel){
	return myCameraList[id]->MemLoadChannel(channel);
}
int	MemSaveChannel_c(CameraID id,int channel){
	return myCameraList[id]->MemSaveChannel(channel);
}

// Store/Retrieve Settings from	system Registry
int	RegLoadSettings_c(CameraID id,const char *pname){
	return myCameraList[id]->RegLoadSettings(pname);
}
int	RegSaveSettings_c(CameraID id,const char *pname){
	return myCameraList[id]->RegSaveSettings(pname);
}

// Raw register	I/O
int	WriteQuadlet_c(CameraID id,unsigned long address,	unsigned long data){
	return myCameraList[id]->WriteQuadlet(address,data);
}
int	ReadQuadlet_c(CameraID id,unsigned long address, unsigned	long *pData){
	return myCameraList[id]->ReadQuadlet(address,pData);
}

// Video format/mode/rate
BOOL HasVideoFormat_c(CameraID id,unsigned long format){
	return myCameraList[id]->HasVideoFormat(format);
}
int	SetVideoFormat_c(CameraID id,unsigned	long format){
	return myCameraList[id]->SetVideoFormat(format);
}
int	GetVideoFormat_c(CameraID id){
	return myCameraList[id]->GetVideoFormat();
}

BOOL HasVideoMode_c(CameraID id,unsigned long	format,	unsigned long mode){
	return myCameraList[id]->HasVideoMode(format,mode);
}
int	SetVideoMode_c(CameraID id,unsigned long mode){
	return myCameraList[id]->SetVideoMode(mode);
}
int	GetVideoMode_c(CameraID id){
	return myCameraList[id]->GetVideoMode();
}

BOOL HasVideoFrameRate_c(CameraID id,unsigned	long format, unsigned long mode, unsigned long rate){
	return myCameraList[id]->HasVideoFrameRate(format,mode,rate);
}
int	SetVideoFrameRate_c(CameraID id,unsigned long	rate){
	return myCameraList[id]->SetVideoFrameRate(rate);
}
int	GetVideoFrameRate_c(CameraID id){
	return myCameraList[id]->GetVideoFrameRate();
}

void GetVideoFrameDimensions_c(CameraID id,unsigned long *pWidth,	unsigned long *pHeight){
	myCameraList[id]->GetVideoFrameDimensions(pWidth,pHeight);
}
void GetVideoDataDepth_c(CameraID id,unsigned short *depth){
	myCameraList[id]->GetVideoDataDepth(depth);
}
bool StatusVideoErrors_c(CameraID id,BOOL Refresh){
	return myCameraList[id]->StatusVideoErrors(Refresh);
}

void UpdateParameters_c(CameraID id,BOOL UpdateOnly){
	myCameraList[id]->UpdateParameters(UpdateOnly);
}

// Image Capture (1394CamCap.cpp)
int	StartImageCapture_c(CameraID id){
	return myCameraList[id]->StartImageCapture();
}
int	CaptureImage_c(CameraID id){
	return myCameraList[id]->CaptureImage();
}
int	StopImageCapture_c(CameraID id){
	return myCameraList[id]->StopImageCapture();
}

// Image Acquisition (1394CamAcq.cpp)
int	StartImageAcquisition_c(CameraID id){
	return myCameraList[id]->StartImageAcquisition();
}
int	StartImageAcquisitionEx_c(CameraID id,int	nBuffers, int FrameTimeout,	int	Flags){
	return myCameraList[id]->StartImageAcquisitionEx(nBuffers,FrameTimeout,Flags);
}
int	AcquireImage_c(CameraID id){
	return myCameraList[id]->AcquireImage();
}
int	AcquireImageEx_c(CameraID id,BOOL	DropStaleFrames, int *lpnDroppedFrames){
	return myCameraList[id]->AcquireImageEx(DropStaleFrames,lpnDroppedFrames);
}
int	StopImageAcquisition_c(CameraID id){
	return myCameraList[id]->StopImageAcquisition();
}
unsigned char *GetRawData_c(CameraID id,unsigned long	*pLength){
	return myCameraList[id]->GetRawData(pLength);
}

//HANDLE GetFrameEvent_c(CameraID id){
//	return myCameraList[id]->
//}

// Video Stream	Control
int	StartVideoStream_c(CameraID id){
	return myCameraList[id]->StartVideoStream();
}
int	StopVideoStream_c(CameraID id){
	return myCameraList[id]->StopVideoStream();
}
bool HasOneShot_c(CameraID id){
	return myCameraList[id]->HasOneShot();
}
int	OneShot_c(CameraID id){
	return myCameraList[id]->OneShot();
}
bool HasMultiShot_c(CameraID id){
	return myCameraList[id]->HasMultiShot();
}
int	MultiShot_c(CameraID id,unsigned short count){
	return myCameraList[id]->MultiShot(count);
}

// Color Format	Conversion (1394CamRGB.cpp)

// convert data	to standard: RGB, upper-left corner
// based on	video format/mode
int	getRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length){
	return myCameraList[id]->getRGB(pBitmap,length);
}

// same	as getRGB, except data is returned in the
// bottom-up, BGR format the MS	calls a	DIB
int	getDIB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length){
	return myCameraList[id]->getDIB(pBitmap,length);
}

// individual RGB converters
int	YtoRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length){
	return myCameraList[id]->YtoRGB(pBitmap,length);
}
int	Y16toRGB_c(CameraID id,unsigned char *pBitmap, unsigned long length){
	return myCameraList[id]->Y16toRGB(pBitmap,length);
}
int	YUV411toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length){
	return myCameraList[id]->YUV411toRGB(pBitmap,length);
}
int	YUV422toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length){
	return myCameraList[id]->YUV422toRGB(pBitmap,length);
}
int	YUV444toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length){
	return myCameraList[id]->YUV444toRGB(pBitmap,length);
}
int	RGB16toRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length){
	return myCameraList[id]->RGB16toRGB(pBitmap, length);
}

// Basic Features Interface
void RefreshControlRegisters_c(CameraID id,BOOL bForceAll){
	return myCameraList[id]->RefreshControlRegisters(bForceAll);
}
bool HasFeature_c(CameraID id,CAMERA_FEATURE fID){
	return myCameraList[id]->HasFeature(fID);
}
bool StatusFeatureError_c(CameraID id,CAMERA_FEATURE fID, BOOL Refresh){
	return myCameraList[id]->StatusFeatureError(fID,Refresh);
}

// NOT POSSIBLE IN C INTERFACE
//C1394CameraControl *GetCameraControl(CAMERA_FEATURE fID);
//C1394CameraControlTrigger *GetCameraControlTrigger_c(CameraID id);
//C1394CameraControlSize *GetCameraControlSize_c(CameraID id);

// Optional Functions
bool HasOptionalFeatures_c(CameraID id){
	return myCameraList[id]->HasOptionalFeatures();
}

// PIO Interface
bool HasPIO_c(CameraID id){
	return myCameraList[id]->HasPIO();
}
unsigned long GetPIOControlOffset_c(CameraID id){
	return myCameraList[id]->GetPIOControlOffset();
}
int GetPIOInputBits_c(CameraID id,unsigned long *ulBits){
	return myCameraList[id]->GetPIOInputBits(ulBits);
}
int GetPIOOutputBits_c(CameraID id,unsigned long *ulBits){
	return myCameraList[id]->GetPIOOutputBits(ulBits);
}
int SetPIOOutputBits_c(CameraID id,unsigned long ulBits){
	return myCameraList[id]->SetPIOOutputBits(ulBits);
}
int GetSIOStatusByte_c(CameraID id,unsigned char *byte){
	return myCameraList[id]->GetSIOStatusByte(byte);
}

// SIO Interface
bool HasSIO_c(CameraID id){
	return myCameraList[id]->HasSIO();
}
unsigned long GetSIOControlOffset_c(CameraID id){
	return myCameraList[id]->GetSIOControlOffset();
}
int SIOConfigPort_c(CameraID id,unsigned long baud, unsigned long databits, unsigned long stopbits, unsigned long parity){
	return myCameraList[id]->SIOConfigPort(baud, databits, stopbits, parity);
}
int SIOEnable_c(CameraID id,BOOL bReceive, BOOL bTransmit){
	return myCameraList[id]->SIOEnable(bReceive, bTransmit);
}
int SIOReadBytes_c(CameraID id,unsigned char *data, unsigned long datalen){
	return myCameraList[id]->SIOReadBytes(data, datalen);
}
int SIOWriteBytes_c(CameraID id,unsigned char *data, unsigned long datalen){
	return myCameraList[id]->SIOWriteBytes(data, datalen);
}

// Strobe Interface
bool HasStrobe_c(CameraID id){
	return myCameraList[id]->HasStrobe();
}
unsigned long GetStrobeControlOffset_c(CameraID id){
	return myCameraList[id]->GetStrobeControlOffset();
}

//C1394CameraControlStrobe *GetStrobeControl(CameraID id,unsigned long strobeID)

// advanced/optional feature offsets
bool HasAdvancedFeature_c(CameraID id){
	return myCameraList[id]->HasAdvancedFeature();
}
unsigned long GetAdvancedFeatureOffset_c(CameraID id){
	return myCameraList[id]->GetAdvancedFeatureOffset();
}

CONTROLHANDLE GetCameraControl_c(CameraID id, CAMERA_FEATURE fID){
	return (CONTROLHANDLE)myCameraList[id]->GetCameraControl(fID);
}

/**************************************
 *  HERE COMES THE CAMERA CONTROL STUFFF
 */

int Inquire_c(CONTROLHANDLE ctrlID,unsigned long *pRawData){
	return ((C1394CameraControl*)ctrlID)->Inquire(pRawData);
}

bool HasPresence_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasPresence();
}

bool HasAbsControl_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasAbsControl();
}

bool HasOnePush_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasOnePush();
}

bool HasReadout_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasReadout();
}

bool HasOnOff_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasOnOff();
}

bool HasAutoMode_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasAutoMode();
}

bool HasManualMode_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->HasManualMode();
}

void GetRange_c(CONTROLHANDLE ctrlID,unsigned short *min, unsigned short *max){
	((C1394CameraControl*)ctrlID)->GetRange(min,max);
}

void GetRangeAbsolute_c(CONTROLHANDLE ctrlID,float *fmin, float *fmax){
	((C1394CameraControl*)ctrlID)->GetRangeAbsolute(fmin,fmax);
}
	
// Status Stuff
int Status_c(CONTROLHANDLE ctrlID,unsigned long *pRawData){
	return ((C1394CameraControl*)ctrlID)->Status(pRawData);
}
// Status Accessors
bool StatusPresence_c(CONTROLHANDLE ctrlID){      ///< Accessor for m_StatusReg.present     \see Status()
	return ((C1394CameraControl*)ctrlID)->StatusPresence();
}

bool StatusAbsControl_c(CONTROLHANDLE ctrlID){    ///< Accessor for m_StatusReg.abscontrol  \see Status()
	return ((C1394CameraControl*)ctrlID)->StatusAbsControl();
}

bool StatusOnOff_c(CONTROLHANDLE ctrlID){         ///< Accessor for m_StatusReg.onoff       \see Status()
	return ((C1394CameraControl*)ctrlID)->StatusOnOff();
}

bool StatusOnePush_c(CONTROLHANDLE ctrlID){      ///< Accessor for m_StatusReg.automode    \see Status()
	return ((C1394CameraControl*)ctrlID)->StatusOnePush();
}

bool StatusAutoMode_c(CONTROLHANDLE ctrlID){      ///< Accessor for m_StatusReg.automode    \see Status()
	return ((C1394CameraControl*)ctrlID)->StatusAutoMode();
}

void GetValue_c(CONTROLHANDLE ctrlID,unsigned short *v_lo, unsigned short *v_hi){
	((C1394CameraControl*)ctrlID)->GetValue(v_lo,v_hi);
}

void GetValueAbsolute_c(CONTROLHANDLE ctrlID,float *f){
	((C1394CameraControl*)ctrlID)->GetValueAbsolute(f);
}

// Status Mutators
int SetAbsControl_c(CONTROLHANDLE ctrlID,BOOL on){ ///< Mutator for m_StatusReg.AbsControl \return Same as SetStatus()
	return ((C1394CameraControl*)ctrlID)->SetAbsControl(on);
}

int SetOnOff_c(CONTROLHANDLE ctrlID,BOOL on){      ///< Mutator for m_StatusReg.onoff      \return Same as SetStatus()
	return ((C1394CameraControl*)ctrlID)->SetOnOff(on);
}

int SetOnePush_c(CONTROLHANDLE ctrlID,BOOL on){    ///< Mutator for m_StatusReg.onepush    \return Same as SetStatus()
	return ((C1394CameraControl*)ctrlID)->SetOnePush(on);
}

int SetAutoMode_c(CONTROLHANDLE ctrlID,BOOL on){   ///< Mutator for m_StatusReg.automode   \return Same as SetStatus()
	return ((C1394CameraControl*)ctrlID)->SetAutoMode(on);
}

int SetValue_c(CONTROLHANDLE ctrlID,unsigned short v_lo, unsigned short v_hi){
	return ((C1394CameraControl*)ctrlID)->SetValue(v_lo, v_hi);
}

int SetValueAbsolute_c(CONTROLHANDLE ctrlID,float f){
	return ((C1394CameraControl*)ctrlID)->SetValueAbsolute(f);
}
	
// Identification Stuff
const char *GetName_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->GetName();
}

const char *GetUnits_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->GetUnits();
}

CAMERA_FEATURE GetFeatureID_c(CONTROLHANDLE ctrlID){
	return ((C1394CameraControl*)ctrlID)->GetFeatureID();
}
