
#ifndef RW_MATH_STATISTICS_HPP
#define RW_MATH_STATISTICS_HPP

#include <list>

namespace rw {
namespace math {

template <class T>
class Statistics {
public:
	/**
	 * @brief Calculates the mean for a list of data
	 */
	static T mean(const std::list<T>& data) {
		T sum = 0;
		BOOST_FOREACH(T d, data) {
			sum += d;
		}
		return (T)sum/data.size();		
	}

	/**
	 * @brief Calculates the variance for a list of data
	 * @param data [in] data 
	 * @param meam [in] The mean value of the data
	 */
	static T variance(const std::list<T>& data, const T& mean) {
		T var = 0;
		BOOST_FOREACH(T d, data) {
			var += Math::sqr(d-mean);
		}
		return var/(data.size()-1);			
	}

	
	/**
	 * Calculates the mean and variance of a list of data
	 */
	static std::pair<T,T> meanAndVariance(const std::list<T>& data) {
		T my = mean(data);
		return std::make_pair(my, Statistics::variance(data, var));
	}

	/**
	 * @brief Returns the mean of the values added 
	 *
	 * The mean is computed as \$\frac{1}{n} \Sigma_{d\in data}d \$
	 */
	T mean() const {
		return Statistics::mean(_data);
	}

	/**
	 * @brief Returns the variance of the values added 
	 * The variance is computed as \$\frac{1}{n-1} \Sigma_{d\in data}(m-\my)^2 \$
	 * where \$\my\$ is the mean of the data.
	 */
	T variance() const {
		return Statistics::variance(_data, Statistics::mean(_data));
	}

	/**
	 * @brief returns the mean and the variance of the data. 
	 *
	 * See documentation of Statistics::mean() and Statistics::variance() 
	 * for how the mean and variane are computed.	 
	 */
	std::pair<T, T> meanAndVariance() const {
		return Statistics::meanAndVariance(_data);
	}

	/**
	 * @brief Add data to statistics
	 */
	void add(const T& t) {
		_data.push_back(t);
	}

	/**
	 * @brief Clear the recorded statistics data
	 */
	void clear() {
		_data.clear();
	}

	/**
	 * @brief Statitics to stream
	 * @param os [in/out] stream to use
	 * @param statistics [in] Statistics to output
	 * @return the resulting stream
	 */
	friend std::ostream& operator<<(std::ostream& os, const Statistics<T>& statistics){
		return os <<"(Mean, Var):{ "<<statistics.mean()<<", "<<statistics.variance()<<"}";
	}

private:
	std::list<T> _data;
};



} } //end namespaces

#endif /* RW_PATH_STATISTICS_HPP */
