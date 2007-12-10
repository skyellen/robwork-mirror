#ifndef TACTILEMATRIX_HPP_
#define TACTILEMATRIX_HPP_

#include <boost/numeric/ublas/matrix.hpp>

namespace rwlibs {
namespace sensors {

    
	class TactileMatrix
	{
	public:
		
		TactileMatrix(unsigned int columns, unsigned int rows):
			_data(columns,rows)
		{
			
		}
		
		virtual ~TactileMatrix(){}
		
		void set(int column, int row, float val){
			_data(column, row) = val;
		}
		
		float get(int column, int row){
			return _data(column, row);
		}
		
		const boost::numeric::ublas::matrix<float>& getMatrix(){
			return _data;
		}
		
		boost::numeric::ublas::matrix<float> _data;
	};

} // sensors
} // rwlibs

#endif /*TACTILEMATRIX_H_*/
