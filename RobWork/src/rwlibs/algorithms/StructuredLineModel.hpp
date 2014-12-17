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
 * @brief A model of a line with data points placed in regular intervals.
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
		 * StructuredLineModels are the same when the distance between them, according
		 * to metric taking into account weighted sum of direction angle difference
		 * and the closest separation between lines is lower than specified threshold.
		 */
		virtual bool same(const StructuredLineModel& model, double threshold) const;
		
		/**
		 * @brief Test if interval matches the dataset.
		 * 
		 * Calculates cumulated distance squared to grid points that would exist for a given interval.
		 * Used for finding interval when refitting. Interval is taken to be the global minimum of this function.
		 */
		double testInterval(const std::vector<rw::math::Vector3D<> >& samples, double interval) const;
		
		/* getters */
		//! @brief Get line.
		rw::geometry::Line line() const { return _line; }
		
		//! @brief Get starting point.
		rw::math::Vector3D<> start() const { return _start; }
		
		//! @brief Get interval.
		double getInterval() const { return _interval; }
		
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
