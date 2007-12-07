/**************************************************************************
**       Title: grab one gray image using libdc1394
**    $RCSfile: GrabTest.cpp,v $
**   $Revision: 1.8 $$Name: rev03 $
**       $Date: 2006-08-11 20:34:53 $
**   Copyright: LGPL $Author: jimali $
** Description:
**
**    Get one gray image using libdc1394 and store it as portable gray map
**    (pgm). Based on 'samplegrab' from Chris Urmson 
**
**-------------------------------------------------------------------------
**
**  $Log: GrabTest.cpp,v $
**  Revision 1.8  2006-08-11 20:34:53  jimali
**  *** empty log message ***
**
**  Revision 1.7  2006/08/11 12:41:12  jimali
**  *** empty log message ***
**
**  Revision 1.6  2006/08/10 14:29:32  jimali
**  *** empty log message ***
**
**  Revision 1.5  2006/08/10 11:53:13  jimali
**  *** empty log message ***
**
**  Revision 1.4  2006/08/08 14:50:38  jimali
**  *** empty log message ***
**
**  Revision 1.3  2006/08/07 15:18:04  jimali
**  *** empty log message ***
**
**  Revision 1.2  2006/08/06 21:42:19  jimali
**  *** empty log message ***
**
**  Revision 1.5  2004/01/20 04:12:27  ddennedy
**  added dc1394_free_camera_nodes and applied to examples
**
**  Revision 1.4  2003/09/02 23:42:36  ddennedy
**  cleanup handle destroying in examples; fix dc1394_multiview to use handle per camera; new example
**
**  Revision 1.3  2001/10/16 09:14:14  ronneber
**  - added more meaningful error message, when no raw1394 handle could be get
**  - does not exit anymore, when camera has no trigger
**
**  Revision 1.2  2001/09/14 08:10:41  ronneber
**  - some cosmetic changes
**
**  Revision 1.1  2001/07/24 13:50:59  ronneber
**  - simple test programs to demonstrate the use of libdc1394 (based
**    on 'samplegrab' of Chris Urmson
**
**
**************************************************************************/

#include <stdio.h>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <stdlib.h>
#include <ipl98/cpp/std_image.h>
#include <points/point3d.h>
#include <points/point2d.h>
#include <sys/io.h>
#include <unistd.h>

#include "ARTagCalibrator.hpp"

/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 0
#define MAX_PORTS   4
#define MAX_CAMERAS 8
#define NUM_BUFFERS 8

/* declarations for libdc1394 */
int numPorts = MAX_PORTS;
raw1394handle_t handles[MAX_CAMERAS];
int numCameras = 0;
dc1394_cameracapture cameras[MAX_CAMERAS];
nodeid_t *camera_nodes;
dc1394_feature_set features;

/* declarations for video1394 */
char *device_name=NULL;

unsigned char *frame_buffer=NULL;

// the address of the parralel port
int par_addr = 0x378;

bool save_image_pgm(const std::string& img_file_name, char* img, int width, int height);
void cleanup(void);
void to_ipl_image(unsigned char* pData, ipl::CStdImage& img);
void setupCameras();
void triggerCameras(int i);
void calibrate_cameras();
void mainLoop();
int imgSubBg(const unsigned char* bg, const unsigned char* img, unsigned char* res, int size,int margin=10);

	/* Capture one frame on all cameras and wait for all frames 
	 * should probably use dc1394_dma_multi_capture_poll so that processing power
	 * is not waisted.  
	 */


int main(int argc, char *argv[]) 
{
	setupCameras();
	
	calibrate_cameras();
	
	mainLoop();	
	
	/* Save both background images to files */
	/*bool succ = save_image_pgm("TEST_BG0.pgm",(char *)bgimage0,cameras[0].frame_width,cameras[0].frame_height) && 
				save_image_pgm("TEST_BG1.pgm",(char *)bgimage1,cameras[1].frame_width,cameras[1].frame_height);
	*/
	cleanup();
	
	return 0;
}

void mainLoop(){
	// setup variables
	unsigned int imgSize = cameras[0].frame_height*cameras[0].frame_width;
  	unsigned char *bgimage0 = new unsigned  char[imgSize];
  	unsigned char *bgimage1 = new unsigned char[imgSize];
  	unsigned char *resimage0 = new unsigned char[imgSize];
//  	unsigned char *resimage1 = new unsigned char[imgSize];
	int lastChange=imgSize, changeCnt=0, CHANGE_THRESHOLD=100,CHANGE_COUNT_THRESHOLD=40;
	// set the trigger low and ready
	triggerCameras(0);
	// search for bgimage
	while(1){
		triggerCameras(3);
		dc1394_dma_multi_capture(cameras, numCameras);
		triggerCameras(0);
		
		int change = imgSubBg((const unsigned char*)bgimage0,(const unsigned char*)cameras[0].capture_buffer,resimage0,imgSize);
		if(lastChange-change>CHANGE_THRESHOLD){
			changeCnt=0; // reset change count
		} else {
			changeCnt++; // the images was allike increase change count
		}
		lastChange = change;
		if(changeCnt>CHANGE_COUNT_THRESHOLD){ 
			// enough pictures have been the same update the background image
			changeCnt = 0;
			memcpy(bgimage0, (unsigned char*)cameras[0].capture_buffer, imgSize);
			memcpy(bgimage1, (unsigned char*)cameras[1].capture_buffer, imgSize);
			std::cout << "background updated!!!" << std::endl;
		}
		
		// find ball in resimage0. if ball found calculate also for resimage1
		/*float x0=0,y0=0;
		if(detectBall(&x,&y)){
			float x1=0,y1=0;
			imgSubBg((const unsigned char*)bgimage1,(const unsigned char*)cameras[1].capture_buffer,resimage1,imgSize);
			
			
		}
		*/
		dc1394_dma_done_with_buffer(&cameras[0]);
		dc1394_dma_done_with_buffer(&cameras[1]);	
	}
}

/* this function substracts a background image (bg) from an image (img) and
 * saves the result in a third image (res). A change is measured. The closer 
 * to (size) the change gets the less change there is in the image.
 */
int imgSubBg(const unsigned char* bg, const unsigned char* img, unsigned char* res, int size,int margin){
	int change = 0;
	for(int i=0;i<size;i++){
		unsigned char pixel = img[i];
		unsigned char bgpixel = bg[i];
		if( bgpixel-margin > pixel ){
			//pixel = (bgpixel-THRESHOLD) -pixel;
		} else if(bgpixel+margin < pixel) {
			//pixel = pixel -(bgpixel+THRESHOLD);
		} else {
			change++;
			pixel = 0;
		}
		res[i] = pixel;
	}
	return change;
}

bool detectBall(unsigned char* src){
	return true;
}


void calibrate_cameras(){
	/* Other declarations */
	int width = cameras[0].frame_width;
	int height = cameras[0].frame_height;
	// place robot in 12 positions
	int artag_id = 0;
	ARTagCalibrator calibCamA(width,height,artag_id);
	ARTagCalibrator calibCamB(width,height,artag_id);
	
	for(int i=0; i<12; i++){
		// place robot in new position
		
		// calculate the position of the center of the artag marker 
		// using robot forward kinematics 
		CPoint3D<float> pt3d;
		// Grap a frame from both cameras
		triggerCameras(3);
		dc1394_dma_multi_capture(cameras, numCameras);
		triggerCameras(0);
		//  the point to the calibrator
		calibCamA.addCalibratePos(pt3d,(unsigned char*)cameras[0].capture_buffer);
		calibCamB.addCalibratePos(pt3d,(unsigned char*)cameras[1].capture_buffer);
		
		/* Release the dma camera buffers */
		for (int j = 0; j < numCameras; j++) {
			dc1394_dma_done_with_buffer(&cameras[j]);
		}
	}
	// Calculate the camera-matrix for each camera
	ipl::CPerspective perspA = calibCamA.getCameraMatrix();
	ipl::CPerspective perspB = calibCamB.getCameraMatrix();
}


void triggerCameras(int i){
	outb(i,par_addr);
	// need small delay
	//outb(0,par_addr);
}

void setupCameras(){
	int fps;
	int res;
	unsigned int channel;
	unsigned int speed;
	int i, p;

	// Check if there is permission to the par-port 	
	if(ioperm(par_addr, 3, 1) == -1) {
    	perror("ioperm");
    	exit(1);
  	}

	raw1394handle_t raw_handle;
	struct raw1394_portinfo ports[MAX_PORTS];

	//get_options(argc,argv);
	fps = FRAMERATE_30;
	res = MODE_640x480_MONO;
	
	/* get the number of ports (cards) */
	raw_handle = raw1394_new_handle();
    if (raw_handle==NULL) {
        perror("Unable to aquire a raw1394 handle\n");
        perror("did you load the drivers?\n");
        exit(-1);
    }
	
	numPorts = raw1394_get_port_info(raw_handle, ports, numPorts);
	raw1394_destroy_handle(raw_handle);
	printf("number of ports = %d\n", numPorts);

	/* get dc1394 handle to each port */
	for (p = 0; p < numPorts; p++)
	{
		int camCount;
		
		/* get the camera nodes and describe them as we find them */
		raw_handle = raw1394_new_handle();
		raw1394_set_port( raw_handle, p );
		camera_nodes = dc1394_get_camera_nodes(raw_handle, &camCount, 1);
		raw1394_destroy_handle(raw_handle);

		/* setup cameras for capture */
		for (i = 0; i < camCount; i++)
		{	
			handles[numCameras] = dc1394_create_handle(p);
			if (handles[numCameras]==NULL) {
				perror("Unable to aquire a raw1394 handle\n");
				perror("did you load the drivers?\n");
				cleanup();
				exit(-1);
			}

			cameras[numCameras].node = camera_nodes[i];
		
			if(dc1394_get_camera_feature_set(handles[numCameras], cameras[numCameras].node, &features) !=DC1394_SUCCESS) 
			{
				printf("unable to get feature set\n");
			} else {
				dc1394_print_feature_set(&features);
			}
		
			dc1394_set_trigger_mode(handles[numCameras],cameras[numCameras].node,TRIGGER_MODE_0);
			dc1394_set_trigger_on_off(handles[numCameras],cameras[numCameras].node,DC1394_TRUE);
			dc1394_set_trigger_polarity(handles[numCameras],cameras[numCameras].node,DC1394_TRUE);
		
			if (dc1394_get_iso_channel_and_speed(handles[numCameras], cameras[numCameras].node,
										 &channel, &speed) != DC1394_SUCCESS) 
			{
				printf("unable to get the iso channel number\n");
				cleanup();
				exit(-1);
			}
			 
			if (dc1394_dma_setup_capture(handles[numCameras], cameras[numCameras].node, i+1 /*channel*/,
									FORMAT_VGA_NONCOMPRESSED, res,
									SPEED_400, fps, NUM_BUFFERS, DROP_FRAMES,
									device_name, &cameras[numCameras]) != DC1394_SUCCESS) 
			{
				fprintf(stderr, "unable to setup camera- check line %d of %s to make sure\n",
					   __LINE__,__FILE__);
				perror("that the video mode,framerate and format are supported\n");
				printf("is one supported by your camera\n");
				cleanup();
				exit(-1);
			}
					
			/*have the camera start sending us data*/
			if (dc1394_start_iso_transmission(handles[numCameras], cameras[numCameras].node) !=DC1394_SUCCESS) 
			{
				perror("unable to start camera iso transmission\n");
				cleanup();
				exit(-1);
			}
			numCameras++;
		}
		dc1394_free_camera_nodes(camera_nodes);
	}

	fflush(stdout);
	if (numCameras < 1) {
		perror("no cameras found :(\n");
		cleanup();
		exit(-1);
	}

	fflush(stdout);
	if (numCameras < 2) {
		perror("This application require at least 2 cameras :(\n");
		cleanup();
		exit(-1);
	}
}

void to_ipl_image(unsigned char* pData, ipl::CStdImage& img){
    ipl::TImage* tim = img.GetTImagePtr();
    ipl::UINT16 bytewidth = tim->ByteWidth;
    for(unsigned int j = 0;j < tim->Height;j++)
      memcpy(tim->ppAllScanLines[j], pData + j*bytewidth, bytewidth );	
}

/*-----------------------------------------------------------------------
 *  save image as 'Image.pgm'
 *-----------------------------------------------------------------------*/
bool save_image_pgm(const std::string& img_file_name, char* img, int width, int height) {
	FILE *imagefile = fopen(img_file_name.c_str(), "w");

	if( imagefile == NULL){
		perror( "Can't create img_file_name");
		return false;
	}

	fprintf(imagefile,"P5\n%u %u 255\n", width, height );
	fwrite((const char *)img, 1, height*width, imagefile);
	fclose(imagefile);
	printf("wrote: img_file_name\n");
	return true;
}

void cleanup(void) {
	int i;
	for (i=0; i < numCameras; i++)
	{
		dc1394_dma_unlisten( handles[i], &cameras[i] );
		dc1394_dma_release_camera( handles[i], &cameras[i]);
		dc1394_destroy_handle(handles[i]);
	}
	if (frame_buffer != NULL)
		free( frame_buffer );
}

