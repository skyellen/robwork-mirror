//#include "stdafx.h"
#include <iostream>
#include <stdlib.h>
#include <iostream>
#include "ARTagCalibrator.hpp"
#include <cmath>

extern "C" {
	#include "artag_rev1g/artag_rev1.h"
}

#define ARTAG_MONO_8 0

static int _num_of_calibrators=0;

ARTagCalibrator::ARTagCalibrator(int width,int height,int id){
	_num_of_calibrators++;
	std::cout << _num_of_calibrators;
	ar_marker_id = id; // the arm id
	cam_width = width;
	cam_height = height;
	
	// Only need to initialize artag once
	if(_num_of_calibrators==1){
		// ARTag initialization 
		if(init_artag(cam_width,cam_height,1)) {
			printf("ERROR: Can't start ARTag\n");
			exit(1);
		} 
	}
}

ARTagCalibrator::~ARTagCalibrator(){
	if(_num_of_calibrators==1)
		close_artag();
}

ipl::CPerspective ARTagCalibrator::getCameraMatrix(){
	ipl::CPerspective persp;
	persp.Calibrate(_pts3d2d);
	return persp;
}

void ARTagCalibrator::reset(){
	_pts3d2d.Empty();	
}		
/*
bool ARTagCalibrator::addCalibratePos(CPoint3D<float> pos, ipl::CStdImage& src){

	//std::cout << "." << state;
	
	// Only 8 bit graylevel images supported 
	if(src.GetBits() != 8){
		std::cout << "Wrong image format:" << src.GetBits() << std::endl;
		return false;
	}
	unsigned char *cam_image = new unsigned char[cam_width*cam_height];
	for(int y=0;y<cam_height;y++)
		for(int x=0;x<cam_width;x++){
			cam_image[y*cam_width+x] = src.GetPixelFast(x,y);
		}
	
	bool res = addCalibratePos(pos,cam_image);
	delete cam_image;
	return res;
}
*/
bool ARTagCalibrator::addCalibratePos(CPoint3D<float> pos, unsigned char* pData){
	
	at_find_objects(pData,ARTAG_MONO_8);
	if(at_object_found(ar_marker_id)){
    	double x, y;
    	at_get_object_position(ar_marker_id,&x,&y);
    	CPoint2D<float> pt2d((float)x,(float)y);
    	_pts3d2d.AddPointSet(pos,pt2d);
    	std::cout << "Artag detected at: " << pt2d << std::endl;
		return true;
	}
	std::cout << "No Artags detected!!!"<< std::endl;
	return false;
}

void drawCross(unsigned char *image,int x, int y, int width){
	unsigned char color = 0;
	image[x+y*width] = color;	
	image[x+1+y*width] = color;
	image[x+1+(y+1)*width] = color;	
	image[x+1+(y-1)*width] = color;
	image[x-1+(y+1)*width] = color;	
	image[x-1+y*width] = color;
	for(int i=x-5;i<x+5;i++)
		image[i+y*width] = color;
//	image[x+y*cam_width] = 0;	
//	image[x+y*cam_width] = 0;
	
}

/************************* Private functions */
void ARTagCalibrator::at_find_objects(unsigned char *image, char rgb){
  
  _num_markers_found = 0;
  
  artag_find_marker(image,rgb,artag_pattern_id,
                    artag_pattern_x0,artag_pattern_y0,artag_pattern_x1,artag_pattern_y1,
                    artag_pattern_x2,artag_pattern_y2,artag_pattern_x3,artag_pattern_y3,
                    &_num_markers_found);
  
/*  printf("%d ARTag markers detected",_num_markers_found);
  for(int i=0;i<_num_markers_found;i++)
   {
    i0=(int)artag_pattern_x0[i];
    j0=(int)artag_pattern_y0[i];
    i1=(int)artag_pattern_x1[i];
    j1=(int)artag_pattern_y1[i];
    i2=(int)artag_pattern_x2[i];
    j2=(int)artag_pattern_y2[i];
    i3=(int)artag_pattern_x3[i];
    j3=(int)artag_pattern_y3[i];
    printf("ARTag marker %d found\n",artag_pattern_id[i]);
    printf("    -corners (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",i0,j0,i1,j1,i2,j2,i3,j3);
   }*/
}

bool ARTagCalibrator::at_object_found(int id){
	for (int i = 0; i < _num_markers_found;i++){
    	if(id == artag_pattern_id[i]) {
    		return(true);
    	}
	}
	return false;
}

void ARTagCalibrator::at_get_object_position(int id, double *x, double *y){
  
  int i;
  for(i = 0; i < _num_markers_found; i++){
    if(id == artag_pattern_id[i])
      break;
  }
  i0=(int)artag_pattern_x0[i];
  j0=(int)artag_pattern_y0[i];
  i1=(int)artag_pattern_x1[i];
  j1=(int)artag_pattern_y1[i];
  i2=(int)artag_pattern_x2[i];
  j2=(int)artag_pattern_y2[i];
  i3=(int)artag_pattern_x3[i];
  j3=(int)artag_pattern_y3[i];
  
  double a1,b1,c1,a2,b2,c2,xcenter,ycenter;
  atdet_endpoints2line((double)i0,(double)j0,(double)i2,(double)j2,&a1,&b1,&c1);
  atdet_endpoints2line((double)i1,(double)j1,(double)i3,(double)j3,&a2,&b2,&c2);
  if(atdet_lines2intersection(a1,b1,c1,a2,b2,c2,&xcenter,&ycenter)!=0) {xcenter=-100.0;ycenter=-100.0;}
  
  *x = xcenter; *y = ycenter;
}

void ARTagCalibrator::atdet_endpoints2line(double x0, double y0, double x1, double y1, double *a, double *b, double *c)
{
  *a=y0-y1; *b=x1-x0; *c=x0*y1-x1*y0;
}


//  artagdet_lines2intersection(a1,b1,c1,a2,b2,c2,&x,&y);
int ARTagCalibrator::atdet_lines2intersection(double a1, double b1, double c1,
                                double a2, double b2, double c2,
                                double *x, double *y)
{
  double denom=a1*b2-a2*b1;
  if(fabs(denom)<0.0001) return -1;
  *x=(b1*c2-b2*c1)/denom;
  *y=(a2*c1-a1*c2)/denom;
  return 0;
}
