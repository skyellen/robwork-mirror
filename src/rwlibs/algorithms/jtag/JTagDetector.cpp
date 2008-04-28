#include "JTagDetector.hpp"

using namespace rw::math;
using namespace rw::sensor;

using namespace rwlibs::algorithms;

namespace {

	double getWeight(const int x, const int y, const Vector2D<>& p){
		double xdiff = x+0.5-p(0);
		double ydiff = y+0.5-p(1);
		double len = xdiff*xdiff + ydiff*ydiff;
		return 2.0-sqrt(len);
	}

	double getSubPixelValue(Image &img, Vector2D<> p){
		// calculates a sub pixel position value from linear interpoaltion between 
		// four neighboring points
		int x = (int)(p(0)-0.5);
		int y = (int)(p(1)-0.5);
		size_t width = img.getWidth();
		unsigned char *imgData = (unsigned char*) img.getImageData();
		double val=0;
		val += imgData[y*width+x]*getWeight(x,y,p);
		val += imgData[y*width+x+1]*getWeight(x+1,y,p);
		val += imgData[(1+y)*width+x]*getWeight(x,y+1,p);
		val += imgData[(1+y)*width+x+1]*getWeight(x+1,y+1,p);
		//val /= 4.0;
		
		return val;
	}
	
	struct Point2D {
		Point2D():_x(0),_y(0){}
		Point2D(int x, int y):_x(x),_y(y){}
		int _x,_y;
	};
}


JTagDetector::JTagDetector(size_t offset_x, size_t offset_y,
						   size_t width, size_t height):
	_width(width),_height(height),
	_contourExt(offset_x, offset_y, width,height),
	_cornerExt(0.2, 4, 4),	
	_thres(100),
	_whiteThres( (int)(_thres*1.2) ),
	_blackThres( (int)(_thres*0.8) ),
	_atEnd(true),
	_x(offset_x),_y(offset_y)
{
}

JTagDetector::~JTagDetector()
{
}

std::vector<JTagMarker> JTagDetector::detectAll(){
	std::vector<int> ids;
	std::vector<JTagMarker> markers;
	//std::cout << "Locating:" << std::endl;
	while( !_atEnd ){
		JTagMarker mark;
		if( !locateNext(_x,_y,mark) )
			continue;
		ids.push_back(mark._id);
		markers.push_back(mark);
	}
	return markers;
}

bool JTagDetector::locateNext(size_t x, size_t y, JTagMarker &mark)
{		
	// get next contour
	//_contourExt.setSearchPos(x,y);
	if(!_contourExt.findNextContour(_contour) ){
		_atEnd=true;
		return false;
	}
	
	// locate corners in the contour
	std::vector<CornerExtractor::Corner> corners;
	if(!_cornerExt.findCorners(*_img,_contour,corners))
		return false;
	
	// if four corners then assume it is a marker
	if( corners.size()!= 4 )
		return false;
	
	// Map corners of a 15*15 matrix onto the corners of the detected marker
	std::vector<Vector2D<> > detRect(4);
	//std::cout << "Detected rectangle: " << std::endl;
	for(size_t i=0;i<4;i++){
		detRect[i] = corners[i].p;
		//std::cout << "Corn: " << detRect[i] << std::endl; 
	}
	std::vector<Vector2D<> > rect15(4);
	rect15[0](0) = 0; rect15[0](1) = 0;
	rect15[1](0) = 15; rect15[1](1) = 0;
	rect15[2](0) = 15; rect15[2](1) = 15;
	rect15[3](0) = 0; rect15[3](1) = 15;
	Vector2D<> centerPoint(15.0/2.0,15.0/2.0);
	
	mark._htf = PerspectiveTransform2D<>::calcTransform(rect15,detRect);
	// htf will transform coordinates in rect15 to coordinates in the detected rect
	
	// calculate position
	mark._pos = mark._htf * centerPoint;
	
	// we are interested in getting color information from the 3x3 bits inside the rectangle
	for(size_t ybit=0; ybit<3; ybit++){
		//std::cout << xbit << " , "; 
		for(size_t xbit=0; xbit<3; xbit++){		
			size_t xoff=xbit*3+3, yoff = ybit*3+3; // each bit is 3 wide plus border (3 wide)
			double bitValSum = 0;
			for(size_t i=xoff; i<xoff+3; i++){
				for(size_t j=yoff; j<yoff+3; j++){
					// we add 0.5 because we want the center pixel value
					Vector2D<> pos = mark._htf * Vector2D<>(i+0.5,j+0.5);
					//std::cout << "\n SUBPIX(" << i <<"," << j << std::endl;
					//std::cout << "\n SUBPIX(" << pos << std::endl;
					bitValSum += getSubPixelValue(*_img, pos);
				}
			}
			// remember to divide by the number of bits
			mark._vals(xbit,ybit) = bitValSum/(9.0*255.0);
			/*
			if( mark.vals(xbit,ybit)<_thres )
				std::cout << 1 ;
			else
				std::cout << 0 ;
			*/
			//std::cout << (int)mark.vals(xbit,ybit) << " , ";
			 
		}
		//std::cout << std::endl ;
	}
	//std::cout << std::endl ;
	
	// Next check if the value matrix of the marker is correct.
	bool isMark = JTagMarker::parseMatrix(mark._vals, mark);
	
	
	/*
	std::vector< Point2D > crs(5);
	crs[0] = Point2D(0,0);
	crs[1] = Point2D(0,2);
	crs[2] = Point2D(2,2);
	crs[3] = Point2D(2,0);
	unsigned char ci0 = mark.vals( crs[2]._x, crs[2]._y );
	unsigned char ci1 = mark.vals( crs[3]._x, crs[3]._y );
	
	for(int i=0; i<4; i++){
		int x = crs[i]._x;
		int y = crs[i]._y;
		
		unsigned char ci2 = mark.vals( x, y );
		if( ci0> _whiteThres && ci1<_blackThres && ci2<_blackThres ){
			//std::cout << "x: " << x << " y: " << y << std::endl;
			// we found (white,black,black) combination	
			// now decode the id
			bool b[6]; // is bit black
			b[2] = mark.vals(1,1)<_thres;
			if(x==0 && y>0){ // last black is in (0,2)
				//std::cout << "(0,2)" << std::endl;
				b[0] = mark.vals(0,1)<_thres;
				b[1] = mark.vals(1,0)<_thres;
				b[4] = mark.vals(2,1)<_thres;
				b[3] = mark.vals(1,2)<_thres;
				
				b[5] = mark.vals(2,2)<_thres;
			} else if( x>0 && y>0 ){ // last black is in (2,2)
				//std::cout << "(2,2)" << std::endl;
				b[1] = mark.vals(0,1)<_thres;
				b[4] = mark.vals(1,0)<_thres;
				b[3] = mark.vals(2,1)<_thres;
				b[0] = mark.vals(1,2)<_thres;
				
				b[5] = mark.vals(2,0)<_thres;				
			} else if( x>0 && y==0 ){ // last black is in (2,0)
				//std::cout << "(2,0)" << std::endl;
				b[4] = mark.vals(0,1)<_thres;
				b[3] = mark.vals(1,0)<_thres;
				b[0] = mark.vals(2,1)<_thres;
				b[1] = mark.vals(1,2)<_thres;
				
				b[5] = mark.vals(0,0)<_thres;								
			} else { // last black is in (0,0)
				//std::cout << "(0,0)" << std::endl;
				b[3] = mark.vals(0,1)<_thres;
				b[0] = mark.vals(1,0)<_thres;
				b[1] = mark.vals(2,1)<_thres;
				b[4] = mark.vals(1,2)<_thres;
				
				b[5] = mark.vals(0,2)<_thres;
			}

			// calculate center of mark
			
			// calculate id			
			unsigned int id = 0;// = b1 + b2<<1 + b3<<2 + b4<<3 + b5<<4 + b6<<5;
			unsigned int val = 1;
			for(int i=0;i<6;i++){
				//std::cout << b[i] << ",";
				if( b[i] )
					id += val;
				val *= 2;
			}
			//std::cout << std::endl;
			//std::cout << "Found marker with ID: " << id << std::endl;
			mark.id = id;
		}
		// remember to update the tail values
		ci0 = ci1;
		ci1 = ci2;
	}
	*/
	
	return isMark;
}
