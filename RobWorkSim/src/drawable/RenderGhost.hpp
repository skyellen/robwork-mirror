#ifndef DRAWABLEGHOST_HPP_
#define DRAWABLEGHOST_HPP_

#include <list>
#include <vector>

#include <rw/kinematics/State.hpp>

#include <rwlibs/drawable/Render.hpp>
#include <rwlibs/drawable/RenderFrame.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>


class RenderGhost: public rwlibs::drawable::Render
{
private:
	std::list<rw::kinematics::Frame*> _frames;
	rwlibs::drawable::WorkCellGLDrawer * _drawer;
	std::vector<rw::kinematics::State> _states;
	
	rwlibs::drawable::RenderFrame *_drawFrame;

public:
		
	RenderGhost(rw::kinematics::Frame *frame, 
				  rwlibs::drawable::WorkCellGLDrawer *drawer);
	
	RenderGhost(std::list<rw::kinematics::Frame*> frames, 
			      rwlibs::drawable::WorkCellGLDrawer *drawer);
	
	virtual ~RenderGhost();
	
	void addState(rw::kinematics::State& state);
	
	void clear();
		
	virtual void draw(DrawType type, double alpha) const;
	
};


#endif /*RenderGhost_HPP_*/
