/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rwlibs_drawable_WorkCellGLDrawer_HPP
#define rwlibs_drawable_WorkCellGLDrawer_HPP

/**
 * @file WorkCellGLDrawer.hpp
 */

#include <vector>
#include <map>

namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class Frame; class State; }}

namespace rwlibs { namespace drawable {

    class Drawable;

    /** @addtogroup drawable */
    /*@{*/

    /**
     * @brief Helps with Visualizing a Workcell. WorkCellGLDrawer is OpenGL specific
     */
    class WorkCellGLDrawer {
    public:
        /**
         * @brief Creates object
         */
        WorkCellGLDrawer(){}

        /**
         * @brief Destroys object
         */
        ~WorkCellGLDrawer();

        /**
         * @brief Draws workcell
         * @param state [in] State for which to draw the WorkCell
         * @param workcell [in] pointer to the workcell being drawn
         */
        void draw(
            const rw::kinematics::State& state,
            rw::models::WorkCell* workcell);

        /**
         * @brief All drawables of the workcell.
         *
         * @param state [in] state of the workcell
         * @param workcell [in] workcell for which the drawables should be returned
         * @return pointers to all drawables in the workcell
         */
        std::vector<Drawable*> getAllDrawables(
            const rw::kinematics::State& state,
            rw::models::WorkCell* workcell);

        /**
         * @brief All Drawables of the subtree of a frame.
         *
         * @param state [in] state of the workcell
         *
         * @param frame [in] subtree for which the drawables should be returned.
         *
         * @param drawables [out] pointers to all drawables in the subtree.
         */
        void getAllDrawables(
            const rw::kinematics::State& state,
            const rw::kinematics::Frame* frame,
            std::vector<Drawable*>& drawables);

        /**
         * @brief This function returns all pointers to drawable objects for a
         * given frame
         *
         * Unlike getAllDrawables() this does not traverse the children of the
         * frame.
         *
         * @param frame [in] the frame to use
         *
         * @return a vector of pointers to drawables of the frame.
         */
        const std::vector<Drawable*>& getDrawablesForFrame(
            const rw::kinematics::Frame* frame);

        /**
         * @brief Adds drawable item to list of drawables for a given frame
         *
         * Ownership of the drawable is taken, i.e. it is deleted upon
         * destruction.
         *
         * @param frame [in] the frame to add to
         *
         * @param drawable [in] drawable to add to list
         */
        void addDrawableToFrame(
            rw::kinematics::Frame* frame,
            Drawable* drawable);

        /**
         * @brief Removes drawable item from a given frame
         *
         * @param frame [in] the frame to remove from
         *
         * @param drawable [in] drawable to remove
         */
        void removeDrawableFromFrame(
            rw::kinematics::Frame* frame,
            Drawable* drawable);

        /**
         * @brief Clears the drawable cache by deleting all drawables
         */
        void clearCache();

        /**
         * @brief Draws camera view
         * @param state [in] State for which to draw the view
         * @param camera [in] pointer to a camera frame
         *
         * This method draws a frame tree as seen from the given camera frame.
         */
        void drawCameraView(
            const rw::kinematics::State& state,
            rw::kinematics::Frame* camera);

    private:
        /**
         * @brief Draws frame and calls recursive to draw all child frames
         * @param frame [in] the frame to draw
         */
        void draw(
            const rw::kinematics::State& state,
            const rw::kinematics::Frame* frame);

        typedef std::vector<Drawable*> DrawableList;

        typedef std::map<const rw::kinematics::Frame*, DrawableList> FrameMap;

        FrameMap _frameMap;

    private:
        WorkCellGLDrawer(const WorkCellGLDrawer&);
        WorkCellGLDrawer& operator=(const WorkCellGLDrawer&);
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
