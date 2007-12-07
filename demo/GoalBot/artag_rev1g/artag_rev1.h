//ARTag Rev 1: 
//
//July 11/2004 by Mark Fiala, National Research Council of Canada
//       email: mark.fiala@nrc-cnrc.gc.ca
//
//-free for non-commercial uses such as research and education
//-this evaluation version times out after Dec 31/2006 - newer evaluation versions will be periodically available from
//the website www.artag.net


//-call init_artag() with the input image dimensions before calling artag_find_marker(),
//     artag_find_marker_white_only, or artagDetectMarker()
//-width,height are dimensions of images passed in detect functions
//-bpp=bytes per pixel=colour depth, bpp is only used by drop-in function artagDetectMarker()
// bpp=1 for greyscale 8-bit, 3=24-bit, 4=32-bit input images
char  init_artag(int width, int height, int bpp);
void  close_artag(void);


//------------------------------  ARToolkit Drop In functions  -----------------------------
//NOTE: uncomment this prototype if you use the function artagDetectMarker.  This allows a seamless insertion of ARTag into 
//      existing ARToolkit programs.  Make sure to also call init_artag() at the top of your program. 
//NOTE: You must have the ARMarkerInfo typdef'd in your program otherwise the compiler will report an error.

//-remove comment /* under here - for use in ARToolkit applications

/*
//detect the markers in the video frame
//if(artagDetectMarker(rgb_cam_image, artoolkit_thresh, &artoolkit_marker_info, &num_markers_found)<0) printf("failed\n");
int  artagDetectMarker(unsigned char *rgb_cam_image, int thresh,
                      ARMarkerInfo **art_marker_info, int *marker_num );
*/

//-remove comment */ above here - for use in ARToolkit applications


int artagLoadPatt(char *patt_name);
//use patt_id=artag_get_id(width,height,name); instead of patt_id=arLoadPatt(patt_name)
//add 1024 to result if you want white-on-black markers
//This function appends a line to a file called <artag_marker_mappings.txt> showing which marker
//number corresponds to that name string

//--note: The image given to ARTag must not be mirrored, when modifying <simple.exe> one
//should comment out the  line "camera.SetFlippedImage(true);" and add the two lines
//"camera.SetFlippedImageVertical(true);" and "camera.SetFlippedImageHorizontal(false);"
//to get SIMPLE.EXE to work in a non-mirrored mode


//-------------- ARTag marker detection without the ARToolkit ARTMarkerInfo structure -------------
//ignore this function if you call the above drop in function


//dataPtr is an unsigned char* containing the input image
//rgb_greybar is a flag: 1=dataPtr is a RGB image, 0=dataPtr is a greyscale image
int  artag_find_marker(unsigned char *dataPtr, char rgb_greybar,
                       int *id,
		       double *x0, double *y0, double *x1, double *y1,
		       double *x2, double *y2, double *x3, double *y3,
		       int *num_markers );


//-------------------------- Choosing ARTag Markers for Application ----------------------------
//use patt_id=artag_get_id(width,height,name); instead of patt_id=arLoadPatt(patt_name)
//call this function repeatedly to get markers in order off of recommended list
//add 1024 to result if you want white-on-black markers
int artag_get_id(void);
//width,height are ignored, they are set by init_artag().

//-artag_get_id() pulls markers off of artag_recommended[] and artag_recommended_hd[] static arrays
//-artag_recommended_marker_pointer is the global that is incremented apon every call.  set to 0
//to start again in the artag_recommended[] list

//-------------------------- Creating ARTag Marker Patterns ----------------------------
//-Call artag_create_marker() to create a bitmap of (10*scale) x (*10*scale) bytes
//artag_create_marker() will fill an unsigned char array with 100*scale*scale bytes
//
//if(artag_create_marker(567,5,image)) problem...  
//create ARTag marker ID=567, 5 pixels/bit, total image will be 50x50 pixels
//this function will malloc room if image is NULL
//returns -1 if problem, 0 otherwise

int  artag_create_marker(int artag_id, int scale, unsigned char *image);



//-------------- Adjusting ARTag Performance Switches -------------
//In the Rev1 release, there are four switches that affect processing and speed, they affect the frame
//rate for slower machines (ARTAG_SWITCH_USE_FULL_RES has the greatest effect), and control what 
//imperfections in the quadrilateral border are allowed.

void artag_set_switch(int switch_name, char setting);    //setting is 1 for on, 0 for off

#define ARTAG_SWITCH_USE_FULL_RES   10   //1=process full resolution (slower), 0=process only 1/2 and 1/4 res.  Default=1
#define ARTAG_SWITCH_BROKEN_SIDE_REPAIR   11 //1=fix broken side not caused by image border (default), 0=don't repair
#define ARTAG_SWITCH_BORDER_CORNER_REPAIR   12 //1=fix quad with missing corner not caused by image border (default)
#define ARTAG_SWITCH_BORDER_3_SIDES   13 //1=fix quad when 4th side is across image border (default), 0=don't repair

//example: to speed up ARTag (by 3x in some cases) but raise false negative rate (times a
//marker is missed):   artag_set_switch(ARTAG_SWITCH_USE_FULL_RES,0);







