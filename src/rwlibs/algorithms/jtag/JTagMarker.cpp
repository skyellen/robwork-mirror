#include "JTagMarker.hpp"

using namespace rwlibs::algorithms;

namespace {
	
	struct Point2D {
		Point2D():_x(0),_y(0){}
		Point2D(int x, int y):_x(x),_y(y){}
		int _x,_y;
	};
	
}

JTagMarker::JTagMarker(int id):
	_id(id),
	_pos(0,0),
	_initialized(false),
	_vals(3,3)
{
	setArray(id);
}

void JTagMarker::setArray(int id){
	// set the 3 corners
	_vals(2,0) = 0.0;
	_vals(0,0) = 1.0;
	_vals(0,2) = 1.0;
	// and then the 
	double b[6];
	int val = 1;
	for(int i=0;i<6;i++){
		if( id & val )
			b[i] = 1.0;
		else 
			b[i] = 0.0;
		val *= 2;
	}	
	_vals(0,1) = b[0];
	_vals(1,0) = b[1];
	_vals(1,1) = b[2];
	_vals(1,2) = b[3];
	_vals(2,1) = b[4];
	_vals(2,2) = b[5];
}


bool JTagMarker::parseMatrix(
    boost::numeric::ublas::matrix<double> vals,
    JTagMarker &mark)
{
	// search counter clock wise for a white, black, black combination.
	std::vector< Point2D > crs(5);
	crs[0] = Point2D(0,0);
	crs[1] = Point2D(0,2);
	crs[2] = Point2D(2,2);
	crs[3] = Point2D(2,0);
	double ci0 = vals( crs[2]._x, crs[2]._y );
	double ci1 = vals( crs[3]._x, crs[3]._y );
	double ci2 = 0;
	for(int i=0; i<4; i++,ci0=ci1,ci1=ci2){// remember to update the tail values
		int x = crs[i]._x;
		int y = crs[i]._y;
		
		ci2 = vals( x, y );
		if( !(ci0<0.5 && ci1>0.5 && ci2>0.5) )
			continue;
				
		//std::cout << "x: " << x << " y: " << y << std::endl;
		// we found (white,black,black) combination	
		// now decode the id
		bool b[6]; // is bit black
		b[2] = vals(1,1)>0.5;
		if(x==0 && y>0){ // last black is in (0,2)
			//std::cout << "(0,2)" << std::endl;
			b[0] = vals(0,1)>0.5;
			b[1] = vals(1,0)>0.5;
			b[4] = vals(2,1)>0.5;
			b[3] = vals(1,2)>0.5;
			
			b[5] = vals(2,2)>0.5;
		} else if( x>0 && y>0 ){ // last black is in (2,2)
			//std::cout << "(2,2)" << std::endl;
			b[1] = vals(0,1)>0.5;
			b[4] = vals(1,0)>0.5;
			b[3] = vals(2,1)>0.5;
			b[0] = vals(1,2)>0.5;
			
			b[5] = vals(2,0)>0.5;				
		} else if( x>0 && y==0 ){ // last black is in (2,0)
			//std::cout << "(2,0)" << std::endl;
			b[4] = vals(0,1)>0.5;
			b[3] = vals(1,0)>0.5;
			b[0] = vals(2,1)>0.5;
			b[1] = vals(1,2)>0.5;
			
			b[5] = vals(0,0)>0.5;								
		} else { // last black is in (0,0)
			//std::cout << "(0,0)" << std::endl;
			b[3] = vals(0,1)>0.5;
			b[0] = vals(1,0)>0.5;
			b[1] = vals(2,1)>0.5;
			b[4] = vals(1,2)>0.5;
			
			b[5] = vals(0,2)>0.5;
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
		
		mark._id = id;
		mark._vals = vals;
		mark._initialized = true;
		return true;
	}
	mark._initialized = false;
	return false;
}
