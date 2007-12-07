#ifndef SYNCCAMERA_H_
#define SYNCCAMERA_H_

#include <stdio.h>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <stdlib.h>
#include <ipl98/cpp/std_image.h>
#include <points/point3d.h>
#include <points/point2d.h>

#include <sys/io.h>
#include <unistd.h>


/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 0
#define MAX_PORTS   4
#define MAX_CAMERAS 8
#define NUM_BUFFERS 8

class SyncCamera
{
	private:
		/* declarations for libdc1394 */
		int numPorts = MAX_PORTS;
		raw1394handle_t handles[MAX_CAMERAS];
		int numCameras = 0;
		dc1394_cameracapture cameras[MAX_CAMERAS];
		nodeid_t *camera_nodes;
		dc1394_feature_set features;
		
		/* declarations for video1394 */
		char *device_name=NULL;
		
		/* Other declarations */
		long frame_length;
		long frame_free;
		int frame=0;
		int adaptor=-1;
		
		int freeze=0;
		int average=0;
		int fps;
		int res;
		unsigned char *frame_buffer=NULL;
	
	public:
		SyncCamera(int triggerMask);
		virtual ~SyncCamera();
		
		/* @brief this static function triggers the capture mechanism of the cameras 
		 * synchronously. It is a non-blocking synchronous capture mechanism.
		 * 
		 * @return  
		 */
		
		get
		static int Trigger();
		
		bool isDataReady();
		
		setBackgroud(unsigned char *frame);
		
		ipl::CStdImage* getFrame();
		
		void setFrame(ipl::CStdImage* frame);
		
	private:
		void setupCamera();
		void cleanup();	
};

#endif /*SYNCCAMERA_H_*/
