	#include "ContactGenerator.hpp"




ContactGenerator::ContactGenerator(const IndexedTriMesh& obj, int nrOfContacts):
    _obj(obj),_nrOfContacts(nrOfContacts)

{
    // calculate object center
    // calculate object bounding sphere

}

std::vector<Contact> ContactGenerator::generateContactSet()
{
    // 1. generate random point "p" inside sphere, close to center point
    // 2. generate two orthogonal vectors from "p" that span some plane
    // 3. shoot N rays from p such that each ray is displaysed
    //    with a small angle to the plane and
    //    such that an angle of about 360/N exist between the vectors when projected to the plane


}
