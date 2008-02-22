#include "JTagGenerator.hpp"

#include <iostream>
#include <fstream>

using namespace rwlibs::algorithms;

#define UNIT " mm "

namespace {
	
	void lineToPS(double x, double y, std::ostream& ostr){
		ostr << x << UNIT << y << UNIT << " lineto" << std::endl;
	}

	void moveToPS(double x, double y, std::ostream& ostr){
		ostr << x << UNIT << y << UNIT << " moveto" << std::endl;
	}
	
	void commentToPS(const std::string& str, std::ostream& ostr){
		ostr << "\% " <<  str << std::endl;
	}
	
	void drawFilledBoxPS(double offx, double offy, double w, std::ostream& ostr){
		
		ostr << "\% Draw a box with width sidelength "<< w << " and offset [" 
			 <<	offx << ";" << offy << "]" << std::endl; 
		ostr << " newpath" << std::endl;
		moveToPS(offx  ,offy  ,ostr);
		lineToPS(offx+w,offy  ,ostr);
		lineToPS(offx+w,offy+w,ostr);
		lineToPS(offx  ,offy+w,ostr);
		ostr << " closepath" << std::endl;
		ostr << " fill" << std::endl;		
	}

}

void JTagGenerator::DrawJTagToPS(int id, const std::string& filename, 
								double width){
	JTagMarker mark(id);
	DrawJTagToPS(mark,filename,width);
}

void JTagGenerator::DrawJTagToPS(const JTagMarker& mark, 
								 const std::string& filename, double width){
	double boxWidth = width/5;
	std::ofstream ostr;
	ostr.open(filename.c_str());
	// first write the header of the PS file
	ostr << "\%!" << std::endl;
	ostr << "\%\% JTag marker with id: " << mark._id
		 << " generated with RobWork JTagMarker generator!" << std::endl;
	ostr << std::endl;
	ostr << "/inch {72 mul} def      \% Convert inches->points (1/72 inch)" << std::endl;
	ostr << "/m {2834.6 mul} def      \% Convert m->points (1/2834.6 m)" << std::endl;
	ostr << "/mm {2.8346 mul} def      \% Convert mm->points (1/2.8346 mm)" << std::endl;
	ostr << std::endl;

	ostr << "\% Set default fill to black!" << std::endl;
	ostr << "0 setgray" << std::endl;
	
	commentToPS("Draw the boundary box and fill it with black",ostr);
	drawFilledBoxPS(20,20,width,ostr);
	
	commentToPS("Now draw the inner white box, so that we get a boundary of width/5",ostr);
	ostr << "gsave" << std::endl;
	ostr << "1 setgray" << std::endl;
	drawFilledBoxPS(20+boxWidth,20+boxWidth,boxWidth*3,ostr);
	ostr << "grestore" << std::endl;
	
	commentToPS("Draw the black boxes acording to 1's in the mark matrix",ostr);
	for(int col=0;col<3;col++){
		for(int row=0;row<3;row++){
			if( mark._vals(col,row)>0.5 ){
				double offx = 20+boxWidth+boxWidth*col;
				double offy = 20+boxWidth+boxWidth*row;
				drawFilledBoxPS(offx,offy,boxWidth,ostr);
			}
		}
	}
	ostr << "showpage" << std::endl;
	//the boundary of the bow
	ostr.close();
}
