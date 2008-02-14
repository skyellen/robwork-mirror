#include "CornerExtractor.hpp"

#include "Line2D.hpp"

#include <math.h>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>

using namespace rw::math;

using namespace rwlibs::algorithms;

#define SEARCH_SPACING 4

namespace {
	
	double getAngle(int ax, int ay, int bx, int by, int cx, int cy){		
		double dot = (cx-bx)*(ax-bx) + (cy-by)*(ay-by);
		double cross =  (ay-by)*(cx-bx) - (cy-by)*(ax-bx);
		return Pi-fabs(atan2(cross,dot));
	}

	bool getNextLine(
			const ContourExtractor::Contour &contour, 
			size_t width, size_t &offset, double thres,
			Line2D &line){
		bool lineStarted=false;
		size_t lineStart;

		int xStart=0,yStart=0;
		double angle1=0,angle2=0;
		int bx(0),by(0);
		for(size_t i=offset; i<contour.size()-(SEARCH_SPACING*2); i+=2){
			int iA = i;
			int iB = i+SEARCH_SPACING;
			int iC = i+SEARCH_SPACING+SEARCH_SPACING;
			
			int ax = contour[iA]%width;
			int ay = contour[iA]/width;
			bx = contour[iB]%width;
			by = contour[iB]/width;
			int cx = contour[iC]%width;
			int cy = contour[iC]/width;
			
			double angle = getAngle(ax,ay,bx,by,cx,cy);
			
			
			if( !lineStarted ){
				//std::cout << "Angle:" << angle << std::endl;
				if( angle < thres ){
					//std::cout << "LINE start: " << ax << " " << ay;
					xStart = ax; 
					yStart = ay;
					lineStarted = true;
					lineStart = i;
					angle1 = angle2 = angle;
				}
			} else {
				double lowangle = (angle+angle1+angle2)/3;
				angle2 = angle1;
				angle1=lowangle;
				if( lowangle > thres ){
					//std::cout << " END: " << bx << " " << by << std::endl;					
					offset = i+1;
					line = Line2D(xStart,yStart,bx,by);
					return true;
				}
			}
		}
		if( lineStarted ){
			//std::cout << " END: " << bx << " " << by << std::endl;					
			offset = contour.size();
			line = Line2D(xStart,yStart,bx,by);
			return true;
		}
		//std::cout << std::endl;
		offset = contour.size();
		return false;
	}
		
}


bool CornerExtractor::findCorners(rw::sensor::Image& srcimg,
				const ContourExtractor::Contour &contour,
				std::vector<Corner> &corners)
{
	// locate all line segments in the contour and extract corners right away
	size_t offset = 0;
	Line2D line(0,0,0,0);
	std::vector<Line2D> lines;
	corners.clear();
	Corner corner;
	Vector2D<> intersection;
	//std::cout << "extracting lines"<< std::endl;
	while( getNextLine(contour,srcimg.getWidth(),offset,0.1,line) ){
		if(lines.size()==0){
			lines.push_back(line);
			continue;
		}
		// check size of line, if very small don't consider it
		//std::cout << "Line Len: " << len << std::endl;
		double len = line.getLength();
		if(  len< 10/*pixels*/)
			continue;
		
		switch( lines.back().getIntersect(line, intersection) ){
		case(Line2D::PARALLEL): // parallel
			return false;
		case(Line2D::COINCIDENT): // parallel and coincident
			// merge lines
			lines.back().p2() = line.p2();
			break;
		case(Line2D::INTERSECTS):
			corner.l1 = lines.back();
			corner.l2 = line;
			corner.p = intersection;
			corners.push_back(corner);
			lines.push_back(line);
			if(corners.size()>_maxNr)
				return false;
			break;
		default:
			return false;
		}
	}
	switch( lines.back().getIntersect(lines.front(), intersection) ){
	case(Line2D::PARALLEL ):
		return false;
	case(Line2D::COINCIDENT):
		lines.front().p1() = lines.back().p1();
		// remove the back element
		lines.resize(lines.size()-1);
		break;
	case(Line2D::INTERSECTS):
		corner.l1 = lines.back();
		corner.l2 = lines.front();
		corner.p = intersection;
		corners.push_back(corner);
	}
	
	if( corners.size()<_minNr || corners.size()>_maxNr ){
		//std::cout << "Nr of lines too small!!" << std::endl;
		return false;
	}
	// now we have th corners.. perhaps do a litle filtering
	return true;
}

