#ifndef ARTAGCALIBRATOR_HPP_
#define ARTAGCALIBRATOR_HPP_

#include <ipl98/cpp/corresponding_3d2d_points.h>
#include <ipl98/cpp/algorithms/perspective.h>
#include <points/point3d.h>
#include <points/point2d.h>
#include <ipl98/cpp/image.h>

#define MAX_NUM_MARKERS 10

class ARTagCalibrator {
	private:
		ipl::CCorresponding3D2DPoints _pts3d2d;
		int cam_width, cam_height, ar_marker_id;
		//double _x0,_y0,_x1,_y1,_x2,_y2,_x3,_y3;
		int _num_markers_found;
	public:
		ARTagCalibrator(int img_width,int img_height, int ar_marker);
		virtual ~ARTagCalibrator();
		
		ipl::CPerspective getCameraMatrix();
		
//		bool addCalibratePos(CPoint3D<float> pos, ipl::CStdImage& image);
		bool addCalibratePos(CPoint3D<float> pos, unsigned char* pData);
		void reset();
	private:
		int i0,j0,i1,j1,i2,j2,i3,j3;

	  	int artag_pattern_id[MAX_NUM_MARKERS];
		double artag_pattern_x0[MAX_NUM_MARKERS],artag_pattern_y0[MAX_NUM_MARKERS];
		double artag_pattern_x1[MAX_NUM_MARKERS],artag_pattern_y1[MAX_NUM_MARKERS];
		double artag_pattern_x2[MAX_NUM_MARKERS],artag_pattern_y2[MAX_NUM_MARKERS];
		double artag_pattern_x3[MAX_NUM_MARKERS],artag_pattern_y3[MAX_NUM_MARKERS];
	
		void at_find_objects(unsigned char *pData, char rgb);
		bool at_object_found(int id);
		void at_get_object_position(int id, double *x, double *y);
		  
		void atdet_endpoints2line(double x0, double y0, double x1, double y1, double *a, double *b, double *c);
		int atdet_lines2intersection(double a1, double b1, double c1,
		                                              double a2, double b2, double c2,
		                                              double *x, double *y);
};



#endif /*ARTAGCALIBRATER_HPP_*/
