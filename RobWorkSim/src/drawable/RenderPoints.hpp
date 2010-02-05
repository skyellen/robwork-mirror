#ifndef RENDERPOINTS_HPP_
#define RENDERPOINTS_HPP_

#include <list>
#include <vector>

#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rw/math/Vector3D.hpp>


class RenderPoints: public rwlibs::drawable::Render
{
public:

	RenderPoints();

	virtual ~RenderPoints();

	/**
	 * @brief adds points to the list of points
	 */
	void addPoints(const std::vector<rw::math::Vector3D<> >& points);
	void addPoint(const rw::math::Vector3D<>& p){ _points.push_back(p); };

	void setColor(double r, double g, double b);

	void clear();

	virtual void draw(DrawType type, double alpha) const;
private:
	std::vector<rw::math::Vector3D<> > _points;
	float _color[3];
};


#endif /*RenderGhost_HPP_*/
