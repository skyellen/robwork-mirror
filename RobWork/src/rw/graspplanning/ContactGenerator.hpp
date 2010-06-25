#ifndef POSEGENERATOR_HPP_
#define POSEGENERATOR_HPP_

#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/sensor/Contact3D.hpp>

namespace rw {
namespace graspplanning {

/**
 * @brief generates candidate contact point sets for grasping a given object.
 *
 * The method used identify
 *
 */
class ContactGenerator {

    /**
     * @brief
     */
    ContactGenerator(const rw::geometry::IndexedTriMesh& obj, int nrOfContacts);

    /**
     * @brief destructor
     */
    virtual ~ContactGenerator(){};

    /**
     * @brief generates a contact set from some heuristic
     */
    std::vector<Contact3D> generateContactSet( );

    Contact3D generateNext();

private:

    const rw::geometry::IndexedTriMesh& _obj;
    const int _nrOfContacts;

    ContactGenerator();
};

}
}
#endif /*POSEGENERATOR_HPP_*/

