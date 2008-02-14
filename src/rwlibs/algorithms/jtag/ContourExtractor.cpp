#include "ContourExtractor.hpp"

#include <rw/common/macros.hpp>

enum Direction { NORTH, EAST, SOUTH, WEST};

using namespace rw::sensor;
using namespace rw::common;

using namespace rwlibs::algorithms; 

namespace {

	bool isItContour(size_t index, unsigned char thres, unsigned char *img){
		return img[index] < thres; 
	}
		
}

void ContourExtractor::reset(rw::sensor::Image& image, double thres){
	// only works on grayscale images
	if( !(image.getColorEncoding()==Image::MONO8 || image.getColorEncoding()==Image::MONO16) ){
		RW_THROW("wrong image type, must be either MONO8 or MONO16");
	}
	// initialize aux vars and state vars	
	_img = (unsigned char*)image.getImageData();
	_thres = (size_t)(255*thres);
	_nextStart = _width+1;
	
	// set border of image so that search is fast
	setImageBorder(255, 1);

	// reset treated array
	_treated.reset();
}

bool ContourExtractor::findNextContour(Contour &contour){
	
	// initialize background state
	bool isBackground=false;
	if( !isItContour( _nextStart ) ){
		isBackground=true;
	}
	
	for( size_t i=_nextStart; i<_width*(_height-1); i++){
		bool isContour = _img[i]<_thres; //isItContour(i, thres, srcImg);
		if( isBackground && isContour ){
			if( _treated[i]==0 ){
//				std::cout << "index: " << i << " Treated: " << _treated[i] << std::endl;			
				_treated[i] = 1;
				followContour(_thres, i, contour );
				if( contour.size() > _minRegionSize ){
					_nextStart = i;
					return true;
				}
			}
		}
		isBackground = !isContour;
	}
	_nextStart = _width*(_height-1);
	return false;
}

#define NORTHIDX i-_width
#define SOUTHIDX i+_width
#define WESTIDX i-1
#define EASTIDX i+1
#define NORTHWESTIDX i-1-_width
#define NORTHEASTIDX i+1-_width
#define SOUTHWESTIDX i-1+_width
#define SOUTHEASTIDX i+1+_width


void ContourExtractor::followContour(
		int thres, 
		size_t start,
		Contour& contour){
	// reset contourdata container and add starting index
	contour.clear();
	contour.push_back(start);
	
	int rot=0;
	Direction direction(NORTH);
		
	// handle first pixel and initialize states from this
	size_t i = start;
	if( !isItContour(NORTHWESTIDX) && !isItContour(NORTHIDX) ){
		direction=EAST;
		if( !isItContour(NORTHEASTIDX) && !isItContour(EASTIDX) ){
			direction=SOUTH;
			if( !isItContour(SOUTHEASTIDX) && !isItContour(EASTIDX) ){
				direction=WEST;					
			}
		}	
	}
	
	do{		
		switch(direction){
		case(NORTH):
			if( isItContour(NORTHWESTIDX) ){
				i = NORTHWESTIDX;
				contour.push_back(i);
				direction=WEST;
				rot--;
			} else if( isItContour(NORTHIDX) ){
				i = NORTHIDX;
				contour.push_back(i);
			} else {
				direction=EAST;
				rot++;
			}
			break;
		case(EAST):
			if( isItContour(NORTHEASTIDX) ){
				i = NORTHEASTIDX;
				contour.push_back(i);
				direction=NORTH;
				rot--;
			} else if( isItContour(EASTIDX) ){
				i = EASTIDX;
				contour.push_back(i);
			} else {
				direction=SOUTH;
				rot++;
			}
			break;
		case(SOUTH):
			if( isItContour(SOUTHEASTIDX) ){
				i = SOUTHEASTIDX;
				contour.push_back(i);
				direction=EAST;
				rot--;
			} else if( isItContour(SOUTHIDX) ){
				i = SOUTHIDX;
				contour.push_back(i);
			} else {
				direction=WEST;
				rot++;
			}
			break;
		case(WEST):
			if( isItContour(SOUTHWESTIDX) ){
				i = SOUTHWESTIDX;
				contour.push_back(i);
				direction=SOUTH;
				rot--;
			} else if( isItContour(WESTIDX) ){
				i = WESTIDX;
				contour.push_back(i);
			} else {
				direction=NORTH;
				rot++;
			}
			break;
		}
//		int y = i/_width;
//		int x = i%_width;
//		std::cout << i << " x:" << x << " y:" << y << " " << _treated[i] << std::endl;
		_treated[i] = 1;
	} while( i!=start );
}

void ContourExtractor::setImageBorder(
					unsigned char bordercolor, 
					int thickness)
{
	// TODO: can be more efficient
	if(thickness==0)
		return;
	for(size_t y=_width; y<_height*_width-1; y+=_width){
		_img[y] = bordercolor;
		_img[y-1] = bordercolor;
	}
	size_t lastIdx = _height*_width-1;
	for(size_t x=0; x<_width; x++){
		_img[x] = bordercolor;
		_img[lastIdx-x] = bordercolor;
	}
}
