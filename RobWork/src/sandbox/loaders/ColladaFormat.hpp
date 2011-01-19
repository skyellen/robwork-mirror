/*
 * ColladaFormat.hpp
 *
 *  Created on: 23/11/2010
 *      Author: jimali
 */

#ifndef COLLADAFORMAT_HPP_
#define COLLADAFORMAT_HPP_

#include <rw/loaders/xml/XercesUtils.hpp>

class ColladaCoreFormat {
public:
    // ************** CORE elements **********************************************
    // Metadata group
    static const XMLCh* AssetId;
    static const XMLCh* COLLADAId;
    static const XMLCh* ContributorId;
    static const XMLCh* GeographicLocationId;

    // Parameters
    static const XMLCh* NewparamId;
    static const XMLCh* ParamId;
    static const XMLCh* SetparamId;

    // Transform
    static const XMLCh* LookatId;
    static const XMLCh* MatrixId;
    static const XMLCh* RotateId;
    static const XMLCh* ScaleId;
    static const XMLCh* SkewId;
    static const XMLCh* TranslateId;

    // library animation
    static const XMLCh* AnimationId;
    static const XMLCh* AnimationClipId;
    static const XMLCh* ChannelId;
    static const XMLCh* InstanceAnimationId;
    static const XMLCh* LibraryAnimationClipsId;
    static const XMLCh* LibraryAnimationsId;
    static const XMLCh* SamplerId;

    // library geometries
    static const XMLCh* ControlVerticesId;
    static const XMLCh* GeometryId;
    static const XMLCh* InstanceGeometryId;
    static const XMLCh* LibraryGeometriesId;
    static const XMLCh* LinesId;
    static const XMLCh* LinestripsId;
    static const XMLCh* MeshId;
    static const XMLCh* PolygonsId;
    static const XMLCh* PolylistId;
    static const XMLCh* SplineId;
    static const XMLCh* TrianglesId;
    static const XMLCh* TrifansId;
    static const XMLCh* TristripsId;
    static const XMLCh* VerticesId;

    // library cameras
    static const XMLCh* CameraId;
    static const XMLCh* ImagerId;
    static const XMLCh* InstanceCameraId;
    static const XMLCh* LibraryCamerasId;
    static const XMLCh* OpticsId;
    static const XMLCh* OrthographicId;
    static const XMLCh* PerspectiveId;

    // library controller
    static const XMLCh* ControllerId;
    static const XMLCh* InstanceControllerId;
    static const XMLCh* JointsId;
    static const XMLCh* LibraryControllersId;
    static const XMLCh* MorphId;
    static const XMLCh* SkeletonId;
    static const XMLCh* SkinId;
    static const XMLCh* TargetsId;
    static const XMLCh* VertexWeightsId;

    // Scene
    static const XMLCh* EvaluateSceneId;
    static const XMLCh* InstanceNodeId;
    static const XMLCh* InstanceVisualSceneId;
    static const XMLCh* LibraryNodesId;
    static const XMLCh* LibraryVisualScenesId;
    static const XMLCh* NodeId;
    static const XMLCh* SceneId;
    static const XMLCh* VisualSceneId;

    // mathematics
    static const XMLCh* FormulaId;
    static const XMLCh* InstanceFormulaId;
    static const XMLCh* LibraryFormulasId;

    // lighting
    static const XMLCh* AmbientId;
    static const XMLCh* ColorId;
    static const XMLCh* DirectionalId;
    static const XMLCh* InstanceLightId;
    static const XMLCh* LibraryLightsId;
    static const XMLCh* LightId;
    static const XMLCh* PointId;
    static const XMLCh* SpotId;

    // Extensibility
    static const XMLCh* ExtraId;
    static const XMLCh* TechniqueId;
    static const XMLCh* TechniqueCommonId;

};

class ColladaEffectsFormat {
public:
    // ************** Effects elements **********************************************
    // Effects group
    static const XMLCh* EffectsId;
    static const XMLCh* BindVertexInputId;
    static const XMLCh* EffectId;
    static const XMLCh* InstanceEffectId;
    static const XMLCh* LibraryEffectsId;
    static const XMLCh* TechniqueId;
    static const XMLCh* TechniqueHintId;

    // Materials
    static const XMLCh* BindId;
    static const XMLCh* BindMaterialsId;
    static const XMLCh* InstanceMaterialId;
    static const XMLCh* LibraryMaterialsId;
    static const XMLCh* MaterialId;


};


#endif /* COLLADAFORMAT_HPP_ */
