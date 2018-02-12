/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_GEOMETRY_ANALYTIC_QUADRATICS_PLAINQUADRATICSHELL_HPP_
#define RW_GEOMETRY_ANALYTIC_QUADRATICS_PLAINQUADRATICSHELL_HPP_

/**
 * @file PlainQuadraticShell.hpp
 *
 * \copydoc rw::geometry::PlainQuadraticShell
 */

#include "QuadraticShell.hpp"

namespace rw {
namespace geometry {
//! @addtogroup geometry

//! @{
/**
 * @brief A collection of concrete Quadratic surface patches, that together form a shell.
 */
class PlainQuadraticShell: public QuadraticShell {
public:
    //! @brief Smart pointer type to PlainQuadraticShell
    typedef rw::common::Ptr<PlainQuadraticShell> Ptr;

	//! @brief Smart pointer type for a const PlainQuadraticShell.
	typedef rw::common::Ptr<const PlainQuadraticShell> CPtr;

    //! @brief Constructor.
	PlainQuadraticShell() {}

	/**
	 * @brief Construct shell from a collection of Quadratic faces.
	 * @param faces [in] collection of Quadratic faces.
	 */
	PlainQuadraticShell(const std::vector<rw::common::Ptr<QuadraticFace> >& faces): _faces(faces) {}

	/**
	 * @brief Copy constructor.
	 * @param shell [in] other shell to copy.
	 */
	PlainQuadraticShell(const PlainQuadraticShell& shell): _faces(shell._faces) {}

	/**
	 * @brief Copy constructor.
	 * @param shell [in] other shell to copy.
	 */
	PlainQuadraticShell(const QuadraticShell& shell) { add(shell); }

	//! @brief Destructor.
	virtual ~PlainQuadraticShell() {}

	//! @copydoc QuadraticShell::isConvex
	virtual bool isConvex() { return false; }

	//! @copydoc QuadraticShell::size
	virtual std::size_t size() const { return _faces.size(); }

	//! @copydoc QuadraticShell::getFace(std::size_t) const
	virtual rw::common::Ptr<const QuadraticFace> getFace(std::size_t idx) const { return _faces[idx]; }

	//! @copydoc QuadraticShell::getFace(std::size_t, QuadraticFace&) const
	virtual void getFace(std::size_t idx, QuadraticFace& dst) const;

	//! @copydoc Shell::getFace(std::size_t, GenericFace&) const
	virtual void getFace(std::size_t idx, GenericFace& face) const;

	/**
	 * @brief Add Quadratic face.
	 * @param face [in] quadratic face to add.
	 */
	void add(const rw::common::Ptr<QuadraticFace> face) {
		_faces.push_back(face);
	}

	/**
	 * @brief Add faces from another shell.
	 * @param shell [in] other shell.
	 */
	void add(const PlainQuadraticShell& shell) {
		_faces.insert(_faces.end(), shell._faces.begin(), shell._faces.end());
	}

	/**
	 * @brief Add faces from another shell.
	 * @param shell [in] other shell.
	 */
	void add(const QuadraticShell& shell) {
		const std::size_t start = _faces.size();
		_faces.resize(shell.size());
		for (std::size_t i = 0; i < shell.size(); i++) {
			shell.getFace(i,*_faces[start+i]);
		}
	}

	//! @brief Remove all faces from the shell.
	void clear() {
		_faces.clear();
	}

	/**
	 * @brief Make a copy of the shell.
	 * @return a new copy.
	 */
	rw::common::Ptr<PlainQuadraticShell> clone() const {
		return rw::common::ownedPtr( new PlainQuadraticShell(*this) );
	}

private:
	virtual rw::common::Ptr<const Face> doGetFace(std::size_t idx) const;

	std::vector<rw::common::Ptr<QuadraticFace> > _faces;
};
//! @}
} /* namespace geometry */
} /* namespace rw */

#endif /* RW_GEOMETRY_ANALYTIC_QUADRATICS_PLAINQUADRATICSHELL_HPP_ */
