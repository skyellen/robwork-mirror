#ifndef TACTILEMATRIX_HPP_
#define TACTILEMATRIX_HPP_

#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>

namespace rwlibs {
namespace sensors {

    
	class TactileMatrix
	{
	public:
		
		TactileMatrix(unsigned int columns, unsigned int rows):
			_data(columns,rows)
		{
			//std::cout << "Tactile ( " << columns << ":" << rows << ")" << std::endl;
			_data = boost::numeric::ublas::zero_matrix<float>(columns,rows);
		}
		
		virtual ~TactileMatrix(){}
		
		void set(int column, int row, float val){
			//std::cout << "Set :" << column << ";"<< row << ";"<< val << std::endl; 
			_data(column, row) = val;
		}
		
		float get(int column, int row){
			return _data(column, row);
		}
		
		const boost::numeric::ublas::matrix<float>& getMatrix() const{
			return _data;
		}
		
		std::string toString() const {
			std::ostringstream ostr;
			for(size_t row=0; row<_data.size2(); row++){
				ostr << row*3.4+1.7 << ";";
				for(size_t col=0; col<_data.size1()-1; col++){
					ostr << _data(col,row) << ";";
				}
				ostr << _data(_data.size1()-1,row) << std::endl;
			}
			return ostr.str();
		}
		
	private:
		
		boost::numeric::ublas::matrix<float> _data;
	};

} // sensors
} // rwlibs

#endif /*TACTILEMATRIX_H_*/
