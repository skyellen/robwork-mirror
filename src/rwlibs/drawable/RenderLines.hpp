#ifndef RWLIBS_DRAWABLE_DRAWABLELINES_HPP_
#define RWLIBS_DRAWABLE_DRAWABLELINES_HPP_

#include "Render.hpp"
#include <rw/math/Vector3D.hpp>

#include <list>

namespace rwlibs { namespace drawable {
    
    /**
     * @brief Render drawing a collection of lines
     */
    class RenderLines: public Render
    {
    public:
        /**
         * @brief Definition of a Line for RenderLines
         */
        typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > Line;

        /**
         * @brief Definition of a list of lines
         */
        typedef std::list<Line> LineList;

        /**
         * @brief Constructs RenderLine with no lines 
         */
        RenderLines();

        /**
         * @brief Construct RenderLine adding the lines specified
         * 
         * @param lines [in] Lines to draw
         */
        RenderLines(const LineList& lines);

        /**
         * @brief Descructor
         */
        virtual ~RenderLines();

        /**
         * @brief Adds a single line to the drawable
         * 
         * @note Each call to addLine generates an update of the display list.
         * Use addLines to add multiple lines with only one update.
         * 
         * @param v1 [in] Start point for line
         * @param v2 [in] End point for line
         */
        void addLine(const rw::math::Vector3D<>& v1, const rw::math::Vector3D<>& v2);

        /**
         * @brief Adds a collection of lines 
         * 
         * After all lines are added, the display list will be updated
         * 
         * @param lines [in] List of lines
         */
        void addLines(const LineList& lines);

        /**
         * @brief Sets the color of the lines.
         * 
         * The influence of the alpha value depends on how opengl is configured.
         * Calling setColor triggers an update of the display list
         * 
         * @param r [in] red [0;1]
         * @param g [in] green [0;1]
         * @param b [in] blue [0;1]
         * @param alpha [in] alpha [0;1]
         */
        void setColor(float r, float g, float b, float alpha);

        /**
         * @brief Sets thickness of the line. 
         * 
         * The thickness is forwarded to glLineWidth. Default 2.0.
         * Calling setThickness triggers an update of the display list
         * @param thickness [in] Thickness of the lines
         */
        void setThickness(float thickness);

        /**
         * @brief Clears all lines
         * 
         * When clearing the lines a new display list without lines will be generated.
         */
        void clear();

        /**
         * @copydoc Render::draw
         */
        void draw(DrawType type, double alpha) const;

    private:
        //Initilized the color and thickness parameters
        void rerender();

        std::string _id;
        LineList _lines;
        GLuint _displayListId;

        float _r;
        float _g;
        float _b;
        float _alpha;
        float _thickness;
    };

}} // end namespaces

#endif // end include guard
