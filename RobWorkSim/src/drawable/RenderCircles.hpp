#ifndef RENDERCIRCLES_HPP_
#define RENDERCIRCLES_HPP_

#include <list>
#include <vector>

#include <util/CircleModel.hpp>

#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rw/math/Vector3D.hpp>


class RenderCircles: public rwlibs::drawable::Render
{
public:

	RenderCircles();

	virtual ~RenderCircles();

	/**
	 * @brief adds points to the list of points
	 */
	void addCircles(const std::vector<CircleModel>& circles);

	void setColor(double r, double g, double b);

	void clear();

	virtual void draw(DrawType type, double alpha) const;
private:
	std::vector<CircleModel> _circles;
	float _color[3];
};


#endif /*RenderGhost_HPP_*/
