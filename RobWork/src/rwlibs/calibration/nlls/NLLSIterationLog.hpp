/*
 * NLLSIterationLog.hpp
 *
 *  Created on: Sep 20, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_
#define RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_

namespace rwlibs {
namespace calibration {

class NLLSIterationLog {
public:
	NLLSIterationLog(int iterationNumber, double conditionNumber, bool isSingular, double residualNorm, double stepNorm, bool isConverged);

	int getIterationNumber() const;

	double getConditionNumber() const;

	bool isSingular() const;

	double getResidualNorm() const;

	double getStepNorm() const;

	bool isConverged() const;

private:
	int _iterationNumber;
	double _conditionNumber;
	bool _isSingular;
	double _residualNorm;
	double _stepNorm;
	bool _isConverged;
};

}
}


#endif /* RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_ */
