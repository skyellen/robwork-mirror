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

#ifndef RWHW_TACTILEMASKMATRIX_HPP
#define RWHW_TACTILEMASKMATRIX_HPP

#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>

namespace rwhw {
    /** @addtogroup tactile */
    /* @{ */

	class TactileMaskMatrix
	{
	public:

		TactileMaskMatrix(unsigned int columns, unsigned int rows):
			_data(columns,rows)
		{
			_data = boost::numeric::ublas::zero_matrix<bool>(columns,rows);
		}

		virtual ~TactileMaskMatrix(){}

		int getWidth(){ return _data.size1(); };

		int getHeight(){ return _data.size2(); };

		void set(int column, int row, bool val){
			_data(column, row) = val;
		}

		bool get(int column, int row)
		{
			return _data(column, row);
		}

		const boost::numeric::ublas::matrix<bool>& getMatrix() const
		{
			return _data;
		}

		std::string toString() const
		{
			std::ostringstream ostr;
			for(size_t row=0; row<_data.size2(); row++)
			{
				for(size_t col=0; col<_data.size1(); col++)
					ostr << (_data(col,row)?"1":"0") << " ";
				ostr << std::endl;
			}
			return ostr.str();
		}

	private:

		boost::numeric::ublas::matrix<bool> _data;
	};

    /* @} */

} // namespace rwhw

#endif /*RWHW_TACTILEMASKMATRIX_HPP*/
