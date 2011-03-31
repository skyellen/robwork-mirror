
#ifndef RW_MATH_STATISTICS_HPP
#define RW_MATH_STATISTICS_HPP

#include <vector>

namespace rw {
namespace math {

template <class T>
class Statistics {
public:
	static std::pair<T, T> normalDistribution(std::vector<T>& data) {
		T sum = 0;
		BOOST_FOREACH(T d, data) {
			sum += d;
		}
		T average = sum / data.length;

		T var = 0;
		BOOST_FOREACH(T d, data) {
			err2 += Math::sqr(d-average);
		}
		return std::make_pair(average, err2/(data.size()-1));	
	}

	/*
	std::pair<Q, Q> normalDistrubitionIndependent(std::vector<Q>& data);
	
*/
T mean() const {
	T sum = 0;
	BOOST_FOREACH(T d, _data) {
		sum += d;
	}
	return sum / data.length;	
}


T variance() const {
	T mean = mean();	

	T var = 0;
	BOOST_FOREACH(T d, _data) {
		var += Math::sqr(d-mean);
	}
	return var /(data.size()-1);	

}

private:
	std::vector<T> _data;
};

} } //end namespaces

#endif /* RW_PATH_STATISTICS_HPP */
