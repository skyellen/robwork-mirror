#ifndef POSEGENERATOR_HPP_
#define POSEGENERATOR_HPP_

namespace rw {
namespace graspplanning {


/**
 * @brief generates poses for grasping some object 
 * 
 */
class PoseGenerator {
    
    /**
     * @brief constructor
     */
    PoseGenerator(const IndexedTriMesh& obj);
    
    /**
     * @brief destructor
     */
    virtual ~PoseGenerator();
    
    /**
     * @brief
     */
    Pose generatePoses(int nrOfPoses);
    
};

}
}


#endif /*POSEGENERATOR_HPP_*/
