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
 
 

#ifndef RW_ALGORITHMS_StructuredLineModel_HPP
#define RW_ALGORITHMS_StructuredLineModel_HPP



/**
 * @file StructuredLineModel.hpp
 */

#include <rw/math/Vector3D.hpp>
#include <rw/geometry/Line.hpp>

#include "RANSACModel.hpp"



namespace rwlibs { namespace algorithms {



/**
 * @brief A model for points placed along a line with regular intervals.
 * 
 * Structured line is defined using a line in 3D, a starting point which is chosen as the data point with lowest (x+y+z) value, and an interval.
 * 
 * @image html sline.png
 * Structured line model requires at least two samples to create.
 */
class StructuredLineModel : public RANSACModel<StructuredLineModel, rw::math::Vector3D<> >
{
	public:
		//! @brief Smart pointer type to this class.
		typedef rw::common::Ptr<StructuredLineModel> Ptr;
		
	public: // constructors
		/**
		 * @brief Constructor.
		 */
		StructuredLineModel() :
			_interval(0.0)
		{}
		
		/**
		 * @brief Constructor.
		 */
		StructuredLineModel(const rw::geometry::Line& line, const rw::math::Vector3D<>& start, double interval) :
			_line(line),
			_start(start),
			_interval(interval)
		{}
		
		//! @brief Destructor.
		virtual ~StructuredLineModel() {}

	public: // methods
		//! @copydoc RANSACModel::fitError
		virtual double fitError(const rw::math::Vector3D<>& sample) const;
		
		//! @copydoc RANSACModel::invalid
		virtual bool invalid() const;
		
		/**
		 * @copydoc RANSACModel::getMinReqData
		 * 
		 * StructuredLineModel requires at least 2 sample.
		 */
		virtual int getMinReqData() const { return 2; }
		
		//! @copydoc RANSACModel::refit
		virtual double refit(const std::vector<rw::math::Vector3D<> >& samples);
		
		/**
		 * @copydoc RANSACModel::same
		 * 
		 * @todo Include interval difference
		 * 
		 * StructuredLineModels are the same when the distance between them, according
		 * to metric taking into account weighted sum of direction angle difference
		 * and the closest separation between lines is lower than specified threshold.
		 */
		virtual bool same(const StructuredLineModel& model, double threshold) const;
		
		/**
		 * @brief Test if interval matches the dataset.
		 * 
		 * This is used to find the interval value for the model when refitting. Returns the total error
		 * associated with given interval. We want to minimize this function, trying a range of possible intervals.
		 * The procedure is as follows:
		 * 1. Find all grid spots on a line between the most outlying samples for a given interval.
		 * 2. For all grid spots find the samples that are closer to it, than they are to any other spot.
		 * 3. If there are no such samples, use the closest one.
		 * 4. For each of the spots calculate cumulated distance to the neighbours.
		 * 5. Total error is the sum of spot errors.
		 */
		double testInterval(const std::vector<rw::math::Vector3D<> >& samples, double interval, rw::math::Vector3D<> start, double maxDist) const;
		
		/* getters */
		//! @brief Get line.
		rw::geometry::Line line() const { return _line; }
		
		//! @brief Get starting point.
		inline rw::math::Vector3D<> start() const { return _start; }
		
		//! @brief Get interval.
		inline double getInterval() const { return _interval; }
		
		/* operators */
		/**
		 * @brief Streaming operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const StructuredLineModel& model)
		{
			return out << "StructuredLine(line: " << model._line << ", start: " << model._start << ", interval: " << model._interval << ")";
		}
	
	protected: // body
		rw::geometry::Line _line;
		rw::math::Vector3D<> _start;
		double _interval;
};



}} // /namespaces

#endif // include guard
