
#ifndef RW_MATH_STATISTICS_HPP
#define RW_MATH_STATISTICS_HPP

#include <list>
#include <boost/foreach.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include "Math.hpp"

namespace rw {
namespace math {

/**
 * @brief Class for collecting data and calculating simple statistics.
 */
template <class T>
class Statistics {
public:
	/**
	 * @brief Calculates the mean for a list of data
	 */
	template <class V>
	static T mean(const V& data) {
		T sum = 0;
		BOOST_FOREACH(T d, data) {
			sum += d;
		}
		return (T)sum/data.size();		
	}

	/**
	 * @brief Calcualtes the m4dian for a list of data
	 */
	template <class V>
	static T median(const V& data) {
		std::vector<T> myvector(data.begin(), data.end());
		std::cout<<"Contructed my vector "<<myvector.size()<<std::endl;
		std::sort(myvector.begin(), myvector.end());
		std::cout<<"Sorted vector"<<std::endl;		
		BOOST_FOREACH(T& d, myvector) {
			std::cout<<"d = "<<d<<std::endl;
		}
		if (myvector.size() == 0)
			return 0;
		if (myvector.size() == 1)
			return myvector.front();

		if (myvector.size() % 1 == 1) {			
			std::cout<<"i1 = "<< std::floor(myvector.size() / 2.0) <<std::endl;
			return myvector[std::floor(myvector.size() / 2.0)];
		} else {
			int i2 = myvector.size() / 2;
			int i1 = i2-1;
			std::cout<<"i1 = "<<i1<<" i2 = "<<i2<<std::endl;
			return (myvector[i1] + myvector[i2])/2.0;
		}
	}


	/**
	 * @brief Calculates the variance for a list of data
	 * @param data [in] data 
	 * @param mean [in] The mean value of the data
	 * @return variance of data
	 */
	template <class V>
	static T variance(const V& data, const T& mean) {
		T var = 0;
		BOOST_FOREACH(T d, data) {
			var += Math::sqr(d-mean);
		}
		return var/(data.size()-1);			
	}

	
	/**
	 * Calculates the mean and variance of a list of data
	 */
	template <class V>
	static std::pair<T,T> meanAndVariance(const V& data) {
		T my = mean(data);
		return std::make_pair(my, Statistics::variance(data, my));
	}

	/** 
	 * @brief Finds the minimal value in \b data
	 */
	template <class V>
	static T minValue(const V& data) {
		if (data.size() == 0)
			return 0;
		T var = data.front();
		BOOST_FOREACH(T d, data) {
			if (d < var)
				var = d;
		}
		return var;
	}

	/** 
	 * @brief Finds the maximal value in \b data
	 */
	template <class V>
	static T maxValue(const V& data) {
		if (data.size() == 0)
			return 0;
		T var = data.front();
		BOOST_FOREACH(T d, data) {
			if (d > var)
				var = d;
		}
		return var;
	}

	/** 
	 * @brief Finds the minimal and maximal value in \b data 
	 */
	template <class V>
	static std::pair<T,T> minAndMaxValue(const V& data) {
		if (data.size() == 0)
			return std::pair<T,T>(0,0);
		T ma = data.front();
		T mi = data.front();
		BOOST_FOREACH(T d, data) {
			if (d < mi)
				mi = d;
			if (d > ma)
				ma = d;
		}
		return std::make_pair(mi, ma);
	}



	/**
	 * @brief Returns the mean of the values added 
	 *
	 * The mean is computed as @f$ \frac{1}{n} \Sigma_{d\in data}d @f$
	 */
	T mean() const {
		return Statistics::mean(_data);
	}

	/**
	 * @brief Returns the median of the values added 
	 *
	 * Given an equal number of element, the mean is calculated as the average of the two center elements.
	 */
	T median() const {
		return Statistics::median(_data);
	}

	/**
	 * @brief Returns the variance of the values added 
	 * The variance is computed as @f$ \frac{1}{n-1} \Sigma_{d\in data}(m-\mu)^2 @f$
	 * where @f$ \mu @f$ is the mean of the data.
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
	 * @brief Returns the minimum value of data added.
	 *
	 * If no data is added 0 is returned. 
	 */
	T minValue() const {
		return Statistics::minValue(_data);
	}

	/**
	 * @brief Returns the maximum value of data added.
	 *
	 * If no data is added 0 is returned.
	 */
	T maxValue() const {
		return Statistics::maxValue(_data);
	}

	/**
	 * @brief Returns pair containing the minimum and maximum value of the data added.
	 *
	 * If no data is added 0 is returned for both values.
	 */
	std::pair<T,T> minAndMaxValue() const {
		return Statistics::minAndMaxValue(_data);
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
	 * @brief Provides reference to the internal data container
	 */
	const std::list<T>& data() const {
		return _data;
	}



	/**
	 * @brief Statitics to stream
	 * @param os [in/out] stream to use
	 * @param statistics [in] Statistics to output
	 * @return the resulting stream
	 */
	friend std::ostream& operator<<(std::ostream& os, const Statistics<T>& statistics){
		std::pair<T, T> meanAndVariance = statistics.meanAndVariance();
		std::pair<T, T> minAndMax = statistics.minAndMaxValue();
		os <<"Mean: "<<meanAndVariance.first<<std::endl;
		os <<"Median: "<<statistics.median()<<std::endl;
		os <<"Variance: "<<meanAndVariance.second<<std::endl;
		os <<"Std Dev: "<<std::sqrt(meanAndVariance.second)<<std::endl;
		os <<"Min Value: "<<minAndMax.first<<std::endl;
		return os <<"Max Value: "<<minAndMax.second<<std::endl;
	}

private:
	std::list<T> _data;
};



} } //end namespaces

#endif /* RW_PATH_STATISTICS_HPP */
