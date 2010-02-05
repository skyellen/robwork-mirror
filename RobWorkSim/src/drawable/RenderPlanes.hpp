#ifndef RENDERPLANES_HPP_
#define RENDERPLANES_HPP_

#include <list>
#include <vector>

#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/Render.hpp>
#include <rw/math/Vector3D.hpp>

#include <util/PlaneModel.hpp>

class RenderPlanes: public rwlibs::drawable::Render
{
public:

	RenderPlanes();

	virtual ~RenderPlanes();

	/**
	 * @brief adds points to the list of points
	 */
	void addPlanes(const std::vector<PlaneModel >& planes);

	void setColor(double r, double g, double b);

	void clear();

	virtual void draw(DrawType type, double alpha) const;
private:
	std::vector<PlaneModel> _planes;
	float _color[3];
};


#endif /*RenderGhost_HPP_*/
