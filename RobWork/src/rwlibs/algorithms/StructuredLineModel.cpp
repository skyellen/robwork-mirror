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
 
 
 
#include "StructuredLineModel.hpp"

#include <rw/math/Metric.hpp>
#include <boost/foreach.hpp>
#include <cmath>



using namespace rwlibs::algorithms;
using namespace rw::math;
using namespace rw::geometry;
using namespace std;
 
 

double StructuredLineModel::fitError(const rw::math::Vector3D<>& sample) const
{
	// 1. find the closest point on line
	Vector3D<> closestPoint = _line.closestPoint(sample);
	
	// 2. find the closest 'grid spot' on the line
	double cells = Math::round((closestPoint - _start).norm2() / _interval);
	Vector3D<> closestSpot = _start + cells * _interval * _line.dir();
	
	// 3. calculate p2p distance of a sample to the closest grid spot
	double distance = (sample - closestSpot).norm2();
	
	return distance;
}



bool StructuredLineModel::invalid() const
{
	// test whether all the data points are co-linear
	return false;
}



double StructuredLineModel::testInterval(const std::vector<rw::math::Vector3D<> >& samples, double interval, rw::math::Vector3D<> start, double maxDist) const
{
	if (interval == 0.0) {
		return std::numeric_limits<double>::max();
	}
	
	double error = 0.0;
	
	// generate as many spots as interval allows in the sample set
	// and sort samples as to which spot they fall closest to
	typedef pair<Vector3D<>, vector<Vector3D<> > > Spot;
	vector<Spot> spots;
	for (double t = 0.0; t < maxDist; t+= interval) {
		Vector3D<> spot = start + t * _line.dir();
		
		// find all samples that are closer to this spot than to neighbours
		vector<Vector3D<> > neighbours;
		BOOST_FOREACH (const Vector3D<>& s, samples) {
			Vector3D<> closestPoint = _line.closestPoint(s);
			if ((closestPoint - spot).norm2() < interval/2.0) {
				neighbours.push_back(s);
			}
		}
		
		// if spot has no neighbours, add the closest sample
		if (neighbours.size() == 0) {
			Vector3D<> nearest = samples[0];
			double ndist = (nearest - spot).norm2();
			BOOST_FOREACH (const Vector3D<>& s, samples) {
				double d = (s - spot).norm2();
				if (d < ndist) {
					nearest = s;
					ndist = d;
				}
			}
			neighbours.push_back(nearest);
		}
		
		spots.push_back(Spot(spot, neighbours));
	}
	
	// for each of the spots calculate the error
	BOOST_FOREACH (const Spot& s, spots) {
		//cout << "neighbours= " << s.second.size() << endl;
		double spot_error = 0.0;
		BOOST_FOREACH (const Vector3D<>& p, s.second) {
			double dist = (p - s.first).norm2();
			spot_error += dist * dist;
		}
		
		error += spot_error;
	}
	
	//cout << "interval= " << interval << " error= " << error << endl;
	
	return error;
}



double StructuredLineModel::refit(const std::vector<rw::math::Vector3D<> >& samples)
{
	if ((int)samples.size() < getMinReqData()) {
		RW_THROW("Too few samples to refit structured line model.");
		return 0.0;
	}
	
	_data = samples;

	// 1. re-fit line
	_line.refit(_data);
	
	// 2. beginning is the point with lowest x+y+z value
	Vector3D<> start = _data[0];
	double xyz = start[0] + start[1] + start[2];
	//cout << "start xyz=" << xyz << endl;
	BOOST_FOREACH (const Vector3D<>& p, _data) {
		double dist = p[0] + p[1] + p[2];
		//cout << "pt xyz=" << p << ": " << dist << endl;
		if (dist < xyz) {
			start = p;
			xyz = dist;
		}
	}
	_start = start;
	
	// find point furthest away from the beginning
	Vector3D<> end = samples[0];
	double dist = 0.0;
	BOOST_FOREACH (const Vector3D<>& p, _data) {
		double d = (p - _start).norm2();
		if (d > dist) {
			end = p;
			dist = d;
		}
	}
	double maxDist = (end - _start).norm2();
	
	// 3. line spacing
	typedef pair<double, double> IntervalQuality;
	vector<IntervalQuality> intervals;
	for (double t = 0.0; t < maxDist; t += maxDist / 1000.0) {
		double q = testInterval(_data, t, _start, maxDist);
		if (Math::isNaN(q)) continue;
		
		intervals.push_back(IntervalQuality(t, q));
		//cout << " --- " << t << ", " << q << endl;
	}
	
	// find interval with best quality
	IntervalQuality bestInterval = intervals[0];
	BOOST_FOREACH (const IntervalQuality& iq, intervals) {
		if (iq.second < bestInterval.second) {
			bestInterval = iq;
		}
	}
	
	_interval = bestInterval.first;
	
	// calculate total fit error
	double error = 0.0;
	for (std::vector<rw::math::Vector3D<> >::iterator i = _data.begin(); i != _data.end(); ++i) {
		double sample_error = fitError(*i);
		error += sample_error * sample_error;
	}
	
	int n = _data.size();
	error /= (n > 0 ? n : 1);
	setQuality(error);
	
	return error;
}



bool StructuredLineModel::same(const StructuredLineModel& model, double threshold) const
{
	// make a metric to compute distance between lines
	rw::math::Metric<rw::geometry::Line>::Ptr metric = rw::geometry::Line::makeMetric();
	
	double d = metric->distance(_line, model._line);
	
	return (d <= threshold);
}
