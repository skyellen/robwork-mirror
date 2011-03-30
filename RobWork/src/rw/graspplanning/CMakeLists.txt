SET(FILES_CPP
    #Contour2DGraspGen.cpp
 	#CG3Grasp2DGen.cpp
	#SDHGraspPlanner2D.cpp
	
	WrenchMeasure3D.cpp
	Contour2DInfoMap.cpp
	
	DiceContactG3D.cpp
	#SBGraspPlanner3D.cpp
		
	ContactValidateFilter.cpp
	GraspValidateFilter.cpp
	CompositeContactFilter.cpp
	PlaneClearanceFilter.cpp
	CurvatureThresFilter.cpp
	SemiForceClosureFilter.cpp
	ContactDistThresFilter.cpp
	CompositeGraspFilter.cpp
	
	GraspQualityMeasure3D.cpp
	CMDistCCPMeasure3D.cpp
	ApproachMeasure3D.cpp
	
	Grasp3D.cpp
	Grasp2D.cpp
	GraspTable.cpp
	GWSMeasure3D.cpp

	#CG3IKSolver2D.cpp
)

SET(FILES_HPP
 	CG3Grasp2DGen.hpp
	SDHGraspPlanner2D.hpp
	WrenchMeasure3D.hpp
	Contour2DInfoMap.hpp
	
	DiceContactG3D.hpp
	SBGraspPlanner3D.hpp
	
	ContactValidateFilter.hpp
	GraspValidateFilter.hpp
	CompositeContactFilter.hpp
	PlaneClearanceFilter.hpp
	CurvatureThresFilter.hpp
	SemiForceClosureFilter.hpp
	ContactDistThresFilter.hpp
	CompositeGraspFilter.hpp
	
	GraspQualityMeasure3D.hpp
	CMDistCCPMeasure3D.hpp
	ApproachMeasure3D.hpp
	
	Grasp3D.hpp
	Grasp2D.hpp
	GraspTable.hpp
	CG3IKSolver2D.hpp
)

SOURCE_GROUP(graspplanning FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${SENSOR_FILES_HPP} DESTINATION "include/rw/sensor")
