#ifndef DRAWABLEPATH_HPP_
#define DRAWABLEPATH_HPP_

#include "Drawable.hpp"

class DrawablePath : public Drawable
{
public:
	DrawablePath(const std::string& filename);
	virtual ~DrawablePath();

protected:
    virtual void update(UpdateType type);
	
};

#endif /*DRAWABLEPATH_HPP_*/
