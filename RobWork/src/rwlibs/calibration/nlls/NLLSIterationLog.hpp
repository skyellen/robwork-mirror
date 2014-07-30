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


#ifndef RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_
#define RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Log entry containing information about an iteration with the NLLS solver
 */
class NLLSIterationLog {
public:
	/**
	 * @brief Constructs log entry
	 */
	NLLSIterationLog(int iterationNumber, double conditionNumber, bool isSingular, double residualNorm, double stepNorm, bool isConverged);

	/**
	 * @brief Number of the iteration
	 */
	int getIterationNumber() const;

	/**
	 * @brief The condition number indicating how well conditioned the equation is
	 */
	double getConditionNumber() const;

	/**
	 * @brief Returns true if the equations are singular
	 */
	bool isSingular() const;

	/**
	 * @brief Returns the norm of the residual
	 */
	double getResidualNorm() const;

	/**
	 * @brief Returns the norm of the step taking in the iteration
	 */
	double getStepNorm() const;

	/**
	 * @brief Returns true if the system has converged according to the criteria of the NLLS solver
	 */
	bool isConverged() const;

private:
	int _iterationNumber;
	double _conditionNumber;
	bool _isSingular;
	double _residualNorm;
	double _stepNorm;
	bool _isConverged;
};

/*@}*/

}
}


#endif /* RWLIBS_CALIBRATION_NLLSITERATIONLOG_HPP_ */
