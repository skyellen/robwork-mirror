/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "MathematicaPlotWidget.hpp"

#include <RobWorkConfig.hpp>

using namespace rwsimlibs::gui;

#if RW_HAVE_MATHEMATICA

#include <rw/common/macros.hpp>
#include <rw/sensor/Image.hpp>
#include <rwlibs/mathematica/Image.hpp>
#include <rwlibs/mathematica/List.hpp>
#include <rwlibs/mathematica/ListPlot.hpp>
#include <rwlibs/mathematica/Mathematica.hpp>
#include <rwlibs/mathematica/Rule.hpp>
#include <rwlibs/mathematica/ReturnPacket.hpp>
#include <rwlibs/mathematica/ToExpression.hpp>

#include <QImage>
#include <QResizeEvent>
#include <QSizePolicy>

using namespace rw::common;
using namespace rwlibs::mathematica;

struct MathematicaPlotWidget::Kernel {
	Kernel(): mathematica(new Mathematica()), initialized(false) {
	}
	~Kernel() {
		link = NULL;
		mathematica->finalize();
		delete mathematica;
	}
	bool isValid() const {
		if (!initialized)
			return false;
		else if (link.isNull())
			return false;
		return link->isOpen();
	}
	void prepare() {
		if (!initialized) {
			if(!mathematica->initialize())
				RW_THROW("Could not initialize Mathematica environment.");
			initialized = true;
		}
		if (!isValid()) {
			link = mathematica->launchKernel();
			RW_ASSERT(!link.isNull());
			RW_ASSERT(link->isOpen());
			Mathematica::Packet::Ptr result;
			*link >> result; // Ignore first In[1]:= prompt
		}
	}
	Mathematica* const mathematica;
	bool initialized;
	Mathematica::Link::Ptr link;
};

struct MathematicaPlotWidget::RenderInfo {
	RenderInfo(): listPlot(NULL) {}
	ListPlot::Ptr listPlot;
	rw::sensor::Image::Ptr rwImg;
	void reset() {
		listPlot = NULL;
		rwImg = NULL;
	}
};

MathematicaPlotWidget::MathematicaPlotWidget(QWidget* parent):
	QLabel(parent),
	_kernel(new Kernel()),
	_render(new RenderInfo())
{
	try {
		_kernel->prepare();
		setStyle();
	} catch(const Exception& e) {
		setText(QString::fromStdString(e.getMessage().getText()));
	}
	setStyleSheet("QLabel { color : red; }");
	setSizePolicy(QSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding));
}

MathematicaPlotWidget::~MathematicaPlotWidget() {
	delete _kernel;
	delete _render;
}

void MathematicaPlotWidget::listPlot(const std::vector<double>& x, const std::vector<double>& y) {
    //std::cout << "plot size: " << width() << " " << height() << std::endl;
	try {
		_kernel->prepare();
		setStyle();
	} catch(const Exception& e) {
		setText(QString::fromStdString(e.getMessage().getText()));
		return;
	}
    std::list<Mathematica::Expression::Ptr> options;
    options.push_back(ownedPtr(new Rule("PlotLabel",ToExpression("Style[\"Test\", font]"))));
    List axes;
    axes.add(ToExpression("Style[\"X axis\", font]"));
    axes.add(ToExpression("Style[\"Y axis\", font]"));
    options.push_back(ownedPtr(new Rule("AxesLabel",axes)));
    //List legends;
    //legends.add(ToExpression("Style[\"Series A\", font]"));
    //legends.add(ToExpression("Style[\"Series B\", font]"));
    //options.push_back(ownedPtr(new Rule("PlotLegends",legends)));
    _render->listPlot = ownedPtr(new ListPlot(x,y,options));
    _render->listPlot->setImageSize(width(),height());
    _render->listPlot->option("AspectRatio",Mathematica::Real(((double)height())/width()));
    _render->listPlot->option("PlotRange",Mathematica::Symbol("All"));
    render();
}

void MathematicaPlotWidget::resizeEvent(QResizeEvent* event) {
    QLabel::resizeEvent(event);
    if(_render->listPlot.isNull())
    	return;
	const int width = event->size().width();
	const int height = event->size().height();
    _render->listPlot->setImageSize(width,height);
    _render->listPlot->option("AspectRatio",Mathematica::Real(((double)height)/width));
    render();
}

void MathematicaPlotWidget::render() {
	const Mathematica::Link& link = *_kernel->link;
	Mathematica::Packet::Ptr result;
	try {
		link << Image(*_render->listPlot);
		link >> result;
		while(result->packetType() != Mathematica::Return)
			link >> result;
	} catch(const Exception& e) {
		setText(QString::fromStdString(e.getMessage().getText()));
		return;
	}
	const ReturnPacket::Ptr imgRet = result.cast<ReturnPacket>();
	if (imgRet.isNull())
		RW_THROW("Expected a return packet!");
	_render->rwImg = Image::toRobWorkImage(*imgRet->expression());
	RW_ASSERT(_render->rwImg->getPixelDepth() == rw::sensor::Image::Depth8U);
	RW_ASSERT(_render->rwImg->getColorEncoding() == rw::sensor::Image::RGB);
	const QImage qImg((const uchar*)_render->rwImg->getImageData(), _render->rwImg->getWidth(), _render->rwImg->getHeight(), 3*_render->rwImg->getWidth(), QImage::Format_RGB888);
	setPixmap(QPixmap::fromImage(qImg));
}

void MathematicaPlotWidget::setStyle() const {
	const Mathematica::Link& link = *_kernel->link;
	Mathematica::Packet::Ptr result;
	std::stringstream style;
	style << "pahblue = CMYKColor[0.78, 0.47, 0.3, 0.05];";
	style << "sdublue = CMYKColor[1, 0.65, 0, 0.3];";
	style << "pahdarkgray = CMYKColor[0, 0, 0, 0.75];";
	style << "pahorange = CMYKColor[0.05, 0.7, 1.0, 0];";
	style << "font={12,pahblue};";
	style << "plotStyle = {PlotStyle -> {{Thickness[0.01], pahorange}, {Thickness[0.01], sdublue}, {Thickness[0.01], pahdarkgray}}, AxesStyle -> pahblue, ImageSize -> {width, Automatic}};";
	link << style.str();
	link >> result; // Ignore return packet
}

#else // if !RW_HAVE_MATHEMATICA

MathematicaPlotWidget::MathematicaPlotWidget(QWidget* parent):
	QLabel(parent),
	_kernel(NULL),
	_render(NULL)
{
}

MathematicaPlotWidget::~MathematicaPlotWidget() {
}

void MathematicaPlotWidget::listPlot(const std::vector<double>& x, const std::vector<double>& y) {
	setText("Compile with Mathematica support for rendering of data.");
}

void MathematicaPlotWidget::resizeEvent(QResizeEvent* event) {
}

void MathematicaPlotWidget::setStyle() const {
}

#endif
