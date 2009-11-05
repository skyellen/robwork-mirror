#ifndef _1394CAMERA_C_H_
#define _1394CAMERA_C_H_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define _AFXDLL

#ifdef MY1394_C_CAMERA_EXPORTS
#define CAMAPIC __declspec(dllexport)
#else
#define CAMAPIC __declspec(dllimport)
#endif

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#ifdef __MINGW32__
#include <windows.h>
#else
#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#include <afxdisp.h>        // MFC Automation classes
#include <afxdtctl.h>		// MFC support for Internet Explorer 4 Common Controls
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT
#endif

#include <1394Camera.h>

#ifdef __cplusplus
extern "C" {
#endif

	typedef int CameraID;

	CameraID CAMAPIC createCameraInstance_c();
	void CAMAPIC releaseCameraInstance_c(CameraID id);

	// Selection/Control
	int	CAMAPIC RefreshCameraList_c(CameraID id);

	int	CAMAPIC InitCamera_c(CameraID id, BOOL reset=FALSE);

	int	CAMAPIC GetNode_c(CameraID id);
	int	CAMAPIC GetNodeDescription_c(CameraID id, int node, char *buf,	int	buflen);
	int	CAMAPIC SelectCamera_c(CameraID id, int node);
	unsigned long CAMAPIC GetVersion_c(CameraID id);
	int	CAMAPIC GetNumberCameras_c(CameraID id);


	void CAMAPIC GetCameraName_c(CameraID id,char	*buf, int len);
	void CAMAPIC GetCameraVendor_c(CameraID id,char *buf,	int	len);
	void CAMAPIC GetCameraUniqueID_c(CameraID id, PLARGE_INTEGER pUniqueID);
	int	 CAMAPIC GetMaxSpeed_c(CameraID id);
	int	 CAMAPIC CheckLink_c(CameraID id);
	bool CAMAPIC HasPowerControl_c(CameraID id);
	bool CAMAPIC StatusPowerControl_c(CameraID id);
	int	 CAMAPIC SetPowerControl(CameraID id,BOOL	on);
	bool CAMAPIC Has1394b_c(CameraID id);
	bool CAMAPIC Status1394b_c(CameraID id);
	int	 CAMAPIC Set1394b_c(CameraID id,BOOL on);

	// Store/Retrieve Settings from	camera EEPROM
	int	CAMAPIC MemGetNumChannels_c(CameraID id);
	int	CAMAPIC MemGetCurrentChannel_c(CameraID id);
	int	CAMAPIC MemLoadChannel_c(CameraID id,int channel);
	int	CAMAPIC MemSaveChannel_c(CameraID id,int channel);

	// Store/Retrieve Settings from	system Registry
	int	CAMAPIC RegLoadSettings_c(CameraID id,const char *pname);
	int	CAMAPIC RegSaveSettings_c(CameraID id,const char *pname);

	// Raw register	I/O
	int	CAMAPIC WriteQuadlet_c(CameraID id,unsigned long address,	unsigned long data);
	int	CAMAPIC ReadQuadlet_c(CameraID id,unsigned long address, unsigned	long *pData);

	// Video format/mode/rate
	BOOL CAMAPIC HasVideoFormat_c(CameraID id,unsigned long format);
	int	CAMAPIC SetVideoFormat_c(CameraID id,unsigned	long format);
	int	CAMAPIC GetVideoFormat_c(CameraID id);

	BOOL CAMAPIC HasVideoMode_c(CameraID id,unsigned long	format,	unsigned long mode);
	int	CAMAPIC SetVideoMode_c(CameraID id,unsigned long mode);
	int	CAMAPIC GetVideoMode_c(CameraID id);

	BOOL CAMAPIC HasVideoFrameRate_c(CameraID id,unsigned	long format, unsigned long mode, unsigned long rate);
	int	CAMAPIC SetVideoFrameRate_c(CameraID id,unsigned long	rate);
	int	CAMAPIC GetVideoFrameRate_c(CameraID id);

	void CAMAPIC GetVideoFrameDimensions_c(CameraID id,unsigned long *pWidth,	unsigned long *pHeight);
	void CAMAPIC GetVideoDataDepth_c(CameraID id,unsigned short *depth);
	bool CAMAPIC StatusVideoErrors_c(CameraID id,BOOL Refresh);

	void CAMAPIC UpdateParameters_c(CameraID id,BOOL UpdateOnly = FALSE);

	// Image Capture (1394CamCap.cpp)
	int	CAMAPIC StartImageCapture_c(CameraID id);
	int	CAMAPIC CaptureImage_c(CameraID id);
	int	CAMAPIC StopImageCapture_c(CameraID id);

	// Image Acquisition (1394CamAcq.cpp)
	int	CAMAPIC StartImageAcquisition_c(CameraID id);
	int	CAMAPIC StartImageAcquisitionEx_c(CameraID id,int	nBuffers, int FrameTimeout,	int	Flags);
	int	CAMAPIC AcquireImage_c(CameraID id);
	int	CAMAPIC AcquireImageEx_c(CameraID id,BOOL	DropStaleFrames, int *lpnDroppedFrames);
	int	CAMAPIC StopImageAcquisition_c(CameraID id);

	unsigned char CAMAPIC *GetRawData_c(CameraID id,unsigned long* pLength);


	//HANDLE GetFrameEvent_c(CameraID id);

	// Video Stream	Control
	int	CAMAPIC StartVideoStream_c(CameraID id);
	int	CAMAPIC StopVideoStream_c(CameraID id);
	bool CAMAPIC HasOneShot_c(CameraID id);
	int	CAMAPIC OneShot_c(CameraID id);
	bool CAMAPIC HasMultiShot_c(CameraID id);
	int	CAMAPIC MultiShot_c(CameraID id,unsigned short count);

	// Color Format	Conversion (1394CamRGB.cpp)

	// convert data	to standard: RGB, upper-left corner
	// based on	video format/mode
	int	CAMAPIC getRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length);

	// same	as getRGB, except data is returned in the
	// bottom-up, BGR format the MS	calls a	DIB
	int	CAMAPIC getDIB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length);

	// individual RGB converters
	int	CAMAPIC YtoRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length);
	int	CAMAPIC Y16toRGB_c(CameraID id,unsigned char *pBitmap, unsigned long length);
	int	CAMAPIC YUV411toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length);
	int	CAMAPIC YUV422toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length);
	int	CAMAPIC YUV444toRGB_c(CameraID id,unsigned char* pBitmap,	unsigned long length);
	int	CAMAPIC RGB16toRGB_c(CameraID id,unsigned	char *pBitmap, unsigned	long length);

	// Basic Features Interface
	void CAMAPIC RefreshControlRegisters_c(CameraID id,BOOL bForceAll = FALSE);
	bool CAMAPIC HasFeature_c(CameraID id,CAMERA_FEATURE fID);
	bool CAMAPIC StatusFeatureError_c(CameraID id,CAMERA_FEATURE fID, BOOL Refresh);

	// NOT POSSIBLE IN C INTERFACE
	//C1394CameraControl *GetCameraControl(CAMERA_FEATURE fID);
	//C1394CameraControlTrigger *GetCameraControlTrigger_c(CameraID id);
	//C1394CameraControlSize *GetCameraControlSize_c(CameraID id);

    // Optional Functions
    bool CAMAPIC HasOptionalFeatures_c(CameraID id);

	// PIO Interface
	bool CAMAPIC HasPIO_c(CameraID id);
	unsigned long CAMAPIC GetPIOControlOffset_c(CameraID id);
	int CAMAPIC GetPIOInputBits_c (CameraID id,unsigned long *ulBits);
	int CAMAPIC GetPIOOutputBits_c(CameraID id,unsigned long *ulBits);
	int CAMAPIC SetPIOOutputBits_c(CameraID id,unsigned long ulBits);
	int CAMAPIC GetSIOStatusByte_c(CameraID id,unsigned char *byte);

	// SIO Interface
	bool CAMAPIC HasSIO_c(CameraID id);
	unsigned long CAMAPIC GetSIOControlOffset_c(CameraID id);
	int CAMAPIC SIOConfigPort_c(CameraID id,unsigned long baud, unsigned long databits, unsigned long stopbits, unsigned long parity);
	int CAMAPIC SIOEnable_c(CameraID id,BOOL bReceive, BOOL bTransmit);
	int CAMAPIC SIOReadBytes_c(CameraID id,unsigned char *data, unsigned long datalen);
	int CAMAPIC SIOWriteBytes_c(CameraID id,unsigned char *data, unsigned long datalen);

	// Strobe Interface
	bool CAMAPIC HasStrobe_c(CameraID id);
	unsigned long CAMAPIC GetStrobeControlOffset_c(CameraID id);
	//C1394CameraControlStrobe *GetStrobeControl(CameraID id,unsigned long strobeID);

	// advanced/optional feature offsets
	bool CAMAPIC HasAdvancedFeature_c(CameraID id);
	unsigned long CAMAPIC GetAdvancedFeatureOffset_c(CameraID id);




#ifdef __cplusplus
}
#endif

#endif // #ifndef _1394CAMERA_C_H_
