/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "HughLineExtractor.hpp"

#include <rw/math/Constants.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/sensor/Image.hpp>

using namespace rw::math;
using namespace rw::sensor;
using namespace rwsim::util;

typedef Vector2D<int> Point;

#define steps 500
#define h_ysteps 500
//static double hugh_threshold = 0.75;
//static double angle_hyst = 0.3;
static double step_size = Pi/steps;

//int buffer[steps][h_ysteps];

namespace {

	int incHughSpace(Image &img, int h_space[], Point &center, int x, int y, double maxdist){
		//std::cout << "inc h space: " << x << " " << y << std::endl;
		double x_r = x-center(0);
		double y_r = y-center(1);
		int max = 0;

		const double h_ystep_scale = h_ysteps/(maxdist*2);

		for(int k=0; k<steps; k++){
			double theta = k*step_size;
			double s = x_r*cos(theta) + y_r*sin(theta) + maxdist;
			/* Beregn afstanden s fra centrum s = x*cos(v)+y*sin(v) */

			if( s>=maxdist*2 || s<0 ){
				std::cout << "s: " << s << " theta: " << theta << std::endl;
				continue;
			}

			int tmp = k;

			/* Make sure the intensity is within limits */
			//std::cout << "[" << tmp << " " << s*steps*h_ystep_scale << "," << maxdist << "]" << std::endl;
			int val = h_space[tmp + (int)(s*h_ystep_scale)*steps ]++;
			if(max<val)
				max = val;
		}
		return max;
	}

}

std::vector<Line2D > HughLineExtractor::extractLines(Image& image){
	Point center(image.getWidth()/2,image.getHeight()/2);
	int max_dist = (int)MetricUtil::norm2(cast<double>(center));
	std::cout << "max_dist: " << max_dist << std::endl;
	std::cout << "center: " << center << std::endl;
	int h_space[steps*h_ysteps];
	for(int i=0; i<steps*h_ysteps;i++)
		h_space[i] = 0;

	int max_value=0;
	for(size_t x=1;x<image.getWidth()-1;x++){
		for(size_t y=1;y<image.getHeight()-1;y++){

			unsigned char color = image.getImageData()[x + (int)( y*image.getWidth() ) ];
			if(color<100)
				continue;

			int val = incHughSpace(image,h_space, center,(int)x,(int)y,max_dist);
			if(max_value<val)
				max_value = val;
		}
	}
	//System.out.println("max: "+max_value);
	//max_value *= hugh_threshold-max_value/5000;
	//return selectLines(h_space,max_value);

	// scaling [0,max_value] -> [0,254]
	double scale = 254.0/max_value;

	std::cout << "Draw hspace: " << max_value << std::endl;
	// for testing, draw the hugh space
	Image himg(steps,h_ysteps, Image::GRAY, Image::Depth8U);
	for(int x=0; x<steps; x++){
		for(int y=0; y<h_ysteps; y++){
			himg.getImageData()[x+y*steps] = (unsigned char)(h_space[x+y*steps]*scale);
			//himg.getImageData()[x+y*steps] = (unsigned char)(h_space[x][y]*scale);
		}
	}
	himg.saveAsPGM("HoughSpace.pgm");
	std::vector<Line2D> lines;
	return lines;
}
