#include "TactilePadItem.hpp"

#include <rwhw/tactile/TactileMaskMatrix.hpp>

#include <boost/foreach.hpp>
#include <QtGui>

using namespace rwhw;

TactilePadItem::TactilePadItem(
	 const TactileMaskMatrix& matrixMask, QMenu *contextMenu,
	 QGraphicsItem *parent, QGraphicsScene *scene):
		 _matrixMask(matrixMask)
{
	_contextMenu = contextMenu;

	int w = _matrixMask.getWidth();
	int h = _matrixMask.getHeight();

	std::vector<QGraphicsRectItem*> rectItems(w*h+1);
	qreal rectW(10.0),rectH(10.0);
	// Create rectangles using the matrixMask
	for(int x=0;x<w;x++){
		for(int y=0;y<w;y++){
			if(_matrixMask.get(x,y)){
				rectItems[x+y*w] = scene->addRect(x*rectW,y*rectH,rectW,rectH);
			} else {
				rectItems[x+y*w] = NULL;
			}
		}
	}
	_rectItems = rectItems;
}


