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

#ifndef RWSIM_CONTACTS_RENDERCONTACTS_HPP_
#define RWSIM_CONTACTS_RENDERCONTACTS_HPP_

#include <rw/graphics/Render.hpp>

/**
 * @file rwsim/contacts/RenderContacts.hpp
 *
 * \copydoc rwsim::contacts::RenderContacts
 */

namespace rwsim {
namespace contacts {
class Contact;

//! @addtogroup rwsim_contacts

//! @{
/**
 * @brief Render for contacts
 */
class RenderContacts: public rw::graphics::Render {
public:
    //! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderContacts> Ptr;

	/**
	 * @brief Construct render with no initial contacts.
	 */
	RenderContacts();

	/**
	 * @brief Constructs a render for a list of contacts.
	 *
	 * @param contacts [in] the list of contacts to draw.
	 */
	RenderContacts(const std::vector<Contact> &contacts);

    /**
     * @brief Destructor
     */
	virtual ~RenderContacts();

	/**
	 * @brief Set which contacts to draw.
	 *
	 * @param contacts [in] contacts to draw.
	 */
	void setContacts(const std::vector<Contact> &contacts);

	/**
	 * @brief Get list of contacts that are currently used by the render.
	 *
	 * @return list of contacts.
	 */
	std::vector<Contact> getContacts() const;

    //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
    void draw(const rw::graphics::DrawableNode::RenderInfo& info,
    		rw::graphics::DrawableNode::DrawType type,
    		double alpha) const;

    /**
     * @brief Sets color of contact points.
     *
     * @param r [in] red color component
     * @param g [in] green color component
     * @param b [in] blue color component
     */
    void setColorPoints(float r, float g, float b);

    /**
     * @brief Sets color of normal arrows.
     *
     * @param r [in] red color component
     * @param g [in] green color component
     * @param b [in] blue color component
     */
    void setColorNormal(float r, float g, float b);

    /**
     * @brief Get color of contact points.
     *
     * @return color of contact point as 3D vector for r-, g-, and b-components.
     */
	rw::math::Vector3D<float> getColorPoint() const;

    /**
     * @brief Get color of contact points.
     *
     * @return color of contact point as 3D vector for r-, g-, and b-components.
     */
	rw::math::Vector3D<float> getColorNormal() const;

private:
	std::vector<Contact> _contacts;
	rw::math::Vector3D<float> _colorPoint, _colorNormal;
	struct GLData;
	const GLData* const _gl;
};
//! @}
} /* namespace contacts */
} /* namespace rwsim */
#endif /* RWSIM_CONTACTS_RENDERCONTACTS_HPP_ */
