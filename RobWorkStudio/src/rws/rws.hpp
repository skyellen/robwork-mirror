/*
 * rws.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: jimali
 */

#ifndef RWS_HPP_
#define RWS_HPP_

#include "AboutBox.hpp"
#include "ArcBallController.hpp"
#include "CameraController.hpp"
#include "FixedAxisController.hpp"
#include "HelpAssistant.hpp"
#include "ImageUtil.hpp"
#include "ImageView.hpp"
#include "RobWorkStudio.hpp"
#include "RobWorkStudioPlugin.hpp"
#include "RWSImageLoaderPlugin.hpp"
#include "RWStudioView3D.hpp"
#include "SceneOpenGLViewer.hpp"
#include "SceneViewerWidget.hpp"

#include "propertyview/PropertyViewDialog.hpp"
#include "propertyview/PropertyViewEditor.hpp"
#include "propertyview/VariantManager.hpp"

#define RWS_USE_RWP_NAMESPACE \
    namespace rws { } \
    namespace rwp \
    { \
        using namespace rws; \
    }

#endif /* RWS_HPP_ */
