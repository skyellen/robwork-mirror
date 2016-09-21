/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_ASSEMBLY_ASSEMBLYRESULT_HPP_
#define RWLIBS_ASSEMBLY_ASSEMBLYRESULT_HPP_

/**
 * @file AssemblyResult.hpp
 *
 * \copydoc rwlibs::assembly::AssemblyResult
 */

#include <rw/trajectory/Path.hpp>

// Forward declarations
namespace rw { namespace trajectory { template <class T> class Timed; }}
namespace rwlibs { namespace task {
template <class T> class Task;
typedef Task<rw::math::Transform3D<> > CartesianTask;
}}

namespace rwlibs {
namespace assembly {

// Forward declarations
class AssemblyState;

//! @addtogroup assembly

//! @{
/**
 * @brief A specification of the result from an execution of an AssemblyTask.
 *
 * The class provides serialization through the CartesianTask format.
 */
class AssemblyResult {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<AssemblyResult> Ptr;

	//! @brief Different error codes.
	typedef enum Error {
		NONE,            //!< No error.
		SIMULATION_ERROR,//!< Failure of physics engine.
		OTHER            //!< Other unspecified error.
	} Error;

    //! @brief Constructor for empty result.
	AssemblyResult();

	/**
	 * @brief Construct result from a CartesianTask representation.
	 * @param task [in] the CartesianTask to construct result from.
	 */
	AssemblyResult(rw::common::Ptr<rwlibs::task::CartesianTask> task);

    //! @brief Destructor.
	virtual ~AssemblyResult();

	/**
	 * @brief Clone the result.
	 * @return a new cloned result.
	 */
	AssemblyResult::Ptr clone() const;

	/**
	 * @brief Convert to the CartesianTask format.
	 * @return a CartesianTask representing the AssemblyResult.
	 */
	rw::common::Ptr<rwlibs::task::CartesianTask> toCartesianTask();

	/**
	 * @brief Store a result to a file.
	 * @param result [in] the result to store.
	 * @param name [in] the file to save to (normally with the extension .assembly.xml).
	 */
	static void saveRWResult(AssemblyResult::Ptr result, const std::string& name);

	/**
	 * @brief Store a list of results to a file.
	 * @param results [in] the list of results to store.
	 * @param name [in] the file to save to (normally with the extension .assembly.xml).
	 */
	static void saveRWResult(std::vector<AssemblyResult::Ptr> results, const std::string& name);

	/**
	 * @brief Load a list of results from a file.
	 * @param name [in] the file to load from.
	 * @return a list of results.
	 */
	static std::vector<AssemblyResult::Ptr> load(const std::string& name);

	/**
	 * @brief Load a list of results from a input stream.
	 * @param inputStream [in] the stream to load from.
	 * @return a list of results.
	 */
	static std::vector<AssemblyResult::Ptr> load(std::istringstream& inputStream);

	/**
	 * @brief Convert an error to string format.
	 * @param error [in] the error to convert.
	 * @return a string representation.
	 */
	static std::string toString(const Error& error);

	/**
	 * @brief Convert a string to a specific error.
	 * @param string [in] the string to convert.
	 * @return an error enum.
	 */
	static Error toError(const std::string& string);

public:
	/**
	 * @name Mandatory settings
	 * @brief These values should always be set in an assembly result.
	 */
	///@{
	//! @brief True or false depending on if the two objects where assembled as specified in the task.
	bool success;
	//! @brief Indication of an error.
	Error error;
	//! @brief The final relative transformation between the objects (with respect to the TCP frames set in the task).
	rw::math::Transform3D<> femaleTmaleEnd;
    ///@}

    /**
     * @name Context & Metadata (optional)
     * @brief Information about the context of the task, and additional information.
     */
    ///@{
	//! @brief The id of the task.
	std::string taskID;
	//! @brief An id of this result.
	std::string resultID;
    ///@}

    /**
     * @name Additional data (optional)
     * @brief Extra detailed data about the trajectory followed.
     */
    ///@{
	//! @brief A TimedPath of AssemblyState objects for the real trajectory.
	rw::trajectory::Path<rw::trajectory::Timed<AssemblyState> > realState;
	//! @brief A TimedPath of AssemblyState objects for the assumed trajectory.
	rw::trajectory::Path<rw::trajectory::Timed<AssemblyState> > assumedState;
	//! @brief The approach pose used.
	rw::math::Transform3D<> approach;
	//! @brief Detailed error message.
	std::string errorMessage;
    ///@}
};
//! @}
} /* namespace assembly */
} /* namespace rwlibs */
#endif /* RWLIBS_ASSEMBLY_ASSEMBLYRESULT_HPP_ */
