#ifndef RWLIBS_DRAWABLE_DRAWABLELINES_HPP_
#define RWLIBS_DRAWABLE_DRAWABLELINES_HPP_

#include "Drawable.hpp"
#include <rw/math/Vector3D.hpp>

#include <list>

namespace rwlibs {
namespace drawable {
    
/**
 * @brief Drawable drawing a collection of lines
 */
class DrawableLines: public Drawable
{
public:
    /**
     * @brief Constructs DrawableLine with no lines 
     */
	DrawableLines();
	
	/**
	 * @brief Construct DrawableLine adding the lines specified
	 * 
	 * @param lines [in] Lines to draw
	 */
	DrawableLines(std::list<std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > >& lines);
	
	/**
	 * @brief Descructor
	 */
	virtual ~DrawableLines();
	
	/**
	 * @brief Adds a single line to the drawable
	 * 
	 * @note Each call to addLine generates an update of the display list. Use addLines to add multiple lines with only one update.
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
	void addLines(const std::list<std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > >& lines);
	
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
protected:
    /**
     * @copydoc Drawable::update
     */
    void update(UpdateType type);
    
private:
    //Initilized the color and thickness parameters
    void initialize();
    
    typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > Line;
    typedef std::list<Line> LineList;

    LineList _lines;
    
    float _r;
    float _g;
    float _b;
    float _alpha;
    float _thickness;
};

} //end namespace drawable
} //end namespace rwlibs


#endif /*RWLIBS_DRAWABLE_DRAWABLELINES_HPP_*/
