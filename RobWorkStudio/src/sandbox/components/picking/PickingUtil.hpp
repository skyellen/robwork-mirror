#ifndef PICKINGUTIL_HPP_
#define PICKINGUTIL_HPP_


/**
 * @brief utility class for performing varius types of picking in
 * the RobWrok studio opengl scene
 */
class PickingUtil {
    
    static double pickDepth(int x, int y);
    
    static Frame* pickFrame(int x, int y);
    
    static std::vector<Frame*> pickVisibleFrames();
    
    
};



#endif /*PICKINGUTIL_HPP_*/
