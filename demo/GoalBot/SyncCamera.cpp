#include "SyncCamera.h"

SyncCamera::SyncCamera()
{
	long frame_length;
	long frame_free;
	int frame=0;
	int adaptor=-1;
	
	int freeze=0;
	int average=0;
	int fps;
	int res;
	unsigned char *frame_buffer=NULL;
}

SyncCamera::~SyncCamera()
{

}

int SyncCamera::Trigger(){
	outb(3,addr);
	outb(0,addr);
}
	
bool SyncCamera::isDataReady();
	
ipl::CStdImage* SyncCamera::getFrame(){
		
}
	
void SyncCamera::setFrame(ipl::CStdImage* frame);

void SyncCamera::setupCamera(){
	unsigned int channel;
	unsigned int speed;
	int i, p;
	raw1394handle_t raw_handle;
	struct raw1394_portinfo ports[MAX_PORTS];

	//get_options(argc,argv);
	fps = FRAMERATE_30;  // FRAMERATE_60
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

void SyncCamera::cleanup(void) {
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

