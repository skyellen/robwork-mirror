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

#ifndef RWLIBS_DRAWABLE_WORKCELLGLDRAWER_HPP
#define RWLIBS_DRAWABLE_WORKCELLGLDRAWER_HPP

/**
 * @file WorkCellGLDrawer.hpp
 */

#include "Drawable.hpp"

#include <vector>
#include <map>
#include <boost/thread/mutex.hpp>

namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class Frame; class State; }}

namespace rwlibs { namespace drawable {

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
            rw::models::WorkCell* workcell,
            unsigned int dmask=Drawable::ALL);

        /**
         * @brief Draws camera view
         * @param state [in] State for which to draw the view
         * @param camera [in] pointer to a camera frame
         * @param dmask [in]
         *
         * This method draws a frame tree as seen from the given camera frame.
         * As default only physical objects in the scene is drawn
         */
        void drawCameraView(
            const rw::kinematics::State& state,
            rw::kinematics::Frame* camera,
            unsigned int dmask=Drawable::Physical);

        /**
         * @brief draws the scene as \b draw but for each frame it draws it pushes
         * its name on the gl name stack.
         *
         * usefull for picking/selecting frames in the scene
         * @param state [in] state that is to be drawn
         * @param workcell [in] the workcell
         */
        void drawAndSelect(const rw::kinematics::State& state,
                           rw::models::WorkCell* workcell,
                           unsigned int dmask=Drawable::ALL);

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

        void lock();

        void unlock();

    private:
        /**
         * @brief Draws frame and calls recursive to draw all child frames
         * @param frame [in] the frame to draw
         */
        void draw(
            const rw::kinematics::State& state,
            const rw::kinematics::Frame* frame,
            unsigned int dmask);

        void drawAndSelect(
            const rw::kinematics::State& state,
            const rw::kinematics::Frame* frame,
            unsigned int dmask);

        typedef std::vector<Drawable*> DrawableList;

        typedef std::map<const rw::kinematics::Frame*, DrawableList> FrameMap;

        FrameMap _frameMap;

        boost::mutex _mutex;
    private:
        WorkCellGLDrawer(const WorkCellGLDrawer&);
        WorkCellGLDrawer& operator=(const WorkCellGLDrawer&);
    };

    /*@}*/

}} // end namespaces

#endif // end include guard
