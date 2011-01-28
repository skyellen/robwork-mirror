/*
 * TactilePadItem.hpp
 *
 *  Created on: 09-03-2009
 *      Author: jimali
 */

#ifndef TACTILEPADITEM_HPP_
#define TACTILEPADITEM_HPP_

#include <rwhw/tactile/TactileMaskMatrix.hpp>
#include <rwhw/tactile/TactileMatrix.hpp>

#include <QtGui>

class TactilePadItem {
public:

	TactilePadItem(
		 const rwhw::TactileMaskMatrix& matrixMask, QMenu *contextMenu,
		 QGraphicsItem *parent, QGraphicsScene *scene);

	void updateSensorValues(const rwhw::TactileMatrix& values);

private:
	std::vector<QGraphicsRectItem*> _rectItems;
	rwhw::TactileMaskMatrix _matrixMask;
	QMenu *_contextMenu;
};

#endif /* TACTILEPADITEM_HPP_ */
