#include "PickingUtil.hpp"

double PickingUtil::pickDepth(int x, int y){
    GLfloat depth;
    int winx = event->x();
    int winy = height()-event->y();
    glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLdouble objx, objy, objz;
    gluUnProject(winx, winy, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);

    return double(depth);
}

Frame* PickingUtil::pickFrame(int x, int y){
    return NULL;
}
    
std::vector<Frame*> PickingUtil::pickVisibleFrames(){
    return std::vector<Frame*>();
}

