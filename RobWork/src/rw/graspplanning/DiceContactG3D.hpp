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

#ifndef RW_GRASPPLANNING_DICECONTACTG3D_HPP_
#define RW_GRASPPLANNING_DICECONTACTG3D_HPP_

#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <vector>

namespace rw { namespace geometry { class TriMesh; } }

namespace rw {
namespace graspplanning {

class ContactValidateFilter;
class Grasp3D;

/**
 * @brief generates candidate contact point sets (contact grasps) for
 * grasping a given object. The nr of contacts per grasp is given by the user
 * and a set of possibly good contact grasps are generated.
 *
 *  The method used is that which is used in the article
 *  "Grasping the Dice by Dicing the Grasp" by Ch. Borst, M. Fisher and
 *  G. Hirzinger
 *
 *  It works by randomly selecting surface contact grasps that are filtered using a
 *  fast but conservative force closure filter.
 *  As such grasps produced is not strictly force closure. Which means
 *  that another filter must also be applied before using the actual grasp.
 *
 * @note (TODO) the sampling of contacts on the surface is not uniform on the surface area
 * but instead in the number of triangles. To enable good sampling the triangles of the
 * geometry should not vary too much in size.
 *
 */
class DiceContactG3D {
public:
    /**
     * @brief
     */
	DiceContactG3D();

    /**
     * @brief destructor
     */
    virtual ~DiceContactG3D(){};

    void setContactFilter(ContactValidateFilter* filter);

    ContactValidateFilter* getContactFilter(){return _cfilter;};

    /**
     * @brief initializes the contact generator on some object.
     * @param obj [in] the object as a indexed triangle mesh
     * @param nrOfContacts [in] the nr of contacts that are allowed in a grasp
     */
    void initialize(const rw::geometry::TriMesh& obj, int nrOfContacts, double mu);

    /**
     * @brief generates a contact set from some heuristic
     */
    std::vector<Grasp3D> generateContactSet(int maxNrOfContacts, double timeout);

    /**
     * @brief generates one contact and returns it
     * @return
     */
    Grasp3D generateNext();

    void setTransform(rw::math::Transform3D<>& t3d){_transform=t3d;};

private:
    const rw::geometry::TriMesh* _obj;
    double _mu;
    int _nrOfContacts;

    std::vector<rw::math::Vector3D<> > _surfNormals;
    rw::math::Transform3D<> _transform;
    ContactValidateFilter* _cfilter;
};

}
}


#endif /*POSEGENERATOR_HPP_*/
