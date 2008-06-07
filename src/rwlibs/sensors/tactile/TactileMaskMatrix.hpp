#ifndef TACTILEMASKMATRIX_HPP_
#define TACTILEMASKMATRIX_HPP_

#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>

namespace rwlibs {
namespace sensors {


	class TactileMaskMatrix
	{
	public:

		TactileMaskMatrix(unsigned int columns, unsigned int rows):
			_data(columns,rows)
		{
			_data = boost::numeric::ublas::zero_matrix<bool>(columns,rows);
		}

		virtual ~TactileMaskMatrix(){}

		void set(int column, int row, bool val)
		{
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

} // sensors
} // rwlibs

#endif /*TACTILEMASKMATRIX_HPP_*/
