#ifndef DRAWABLEPATH_HPP_
#define DRAWABLEPATH_HPP_

#include "Render.hpp"

class RenderPath : public Render
{
public:
	RenderPath(const std::string& filename);
	virtual ~RenderPath();

protected:
    virtual void update(UpdateType type);
	
};

#endif /*DRAWABLEPATH_HPP_*/
