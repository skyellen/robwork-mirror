SET(FILES_CPP
  WorkCellLoader.cpp

  model3d/LoaderAC3D.cpp
  model3d/Loader3DS.cpp
  model3d/Model3DS.cpp
  model3d/LoaderOBJ.cpp
  model3d/LoaderTRI.cpp
  model3d/STLFile.cpp
  Model3DFactory.cpp
  Model3DLoader.cpp
  GeometryFactory.cpp
  GeometryLoader.cpp

  path/PathLoader.cpp
  tul/Tag.cpp
  tul/TULLoader.cpp

  rwxml/DependencyGraph.cpp
  rwxml/MultipleFileIterator.cpp
  rwxml/XMLParserUtil.cpp
  rwxml/XMLRWPreParser.cpp
  rwxml/XMLRWParser.cpp
  rwxml/XMLRWLoader.cpp
  rwxml/XMLErrorHandler.cpp
  rwxml/XMLParser.cpp
  rwxml/XML.cpp
  
  dom/DOMBasisTypes.cpp
  dom/DOMPropertyMapFormat.cpp
  dom/DOMPropertyMapLoader.cpp
  dom/DOMPropertyMapSaver.cpp
  dom/DOMPathLoader.cpp
  dom/DOMPathSaver.cpp
  dom/DOMTrajectoryLoader.cpp
  dom/DOMTrajectorySaver.cpp
  dom/DOMProximitySetupLoader.cpp
  
  colsetup/CollisionSetupLoader.cpp
  
  image/PGMLoader.cpp
  image/PPMLoader.cpp
  image/RGBLoader.cpp
  ImageLoader.cpp
)

SET(FILES_HPP
  WorkCellLoader.hpp
  
  path/PathLoader.hpp
  tul/Tag.hpp
  tul/TULLoader.hpp

  rwxml/DependencyGraph.hpp
  rwxml/MultipleFileIterator.hpp
  rwxml/XMLParserUtil.hpp
  rwxml/XMLRWPreParser.hpp
  rwxml/XMLRWParser.hpp
  rwxml/XMLRWLoader.hpp
  rwxml/XMLErrorHandler.hpp
  rwxml/XMLParser.hpp
  rwxml/XML.hpp
  
  dom/DOMBasisTypes.hpp
  dom/DOMPropertyMapFormat.hpp
  dom/DOMPropertyMapLoader.hpp
  dom/DOMPropertyMapSaver.hpp
  dom/DOMPathLoader.hpp
  dom/DOMPathSaver.hpp
  dom/DOMTrajectoryLoader.hpp
  dom/DOMTrajectorySaver.hpp
  dom/DOMProximitySetupLoader.hpp

  colsetup/CollisionSetupLoader.hpp
  image/PGMLoader.hpp
  image/PPMLoader.hpp
  image/RGBLoader.hpp
  ImageLoader.hpp
)

IF (RW_HAVE_ASSIMP)
	set(FILES_CPP ${FILES_CPP}
  		model3d/LoaderAssimp.cpp
  	)

    set(FILES_HPP ${FILES_HPP}
  		model3d/LoaderAssimp.hpp
    )
ENDIF()

IF (RW_HAVE_XERCES) 
	set(FILES_CPP ${FILES_CPP}
    	xml/XercesErrorHandler.cpp
      	xml/XercesUtils.cpp
      	xml/XMLBasisTypes.cpp
      	xml/XMLPathFormat.cpp
      	xml/XMLPathLoader.cpp
      	xml/XMLPathSaver.cpp
      	xml/XMLTrajectoryFormat.cpp
      	xml/XMLTrajectoryLoader.cpp
      	xml/XMLTrajectorySaver.cpp
      	xml/XMLPropertyFormat.cpp
      	xml/XMLPropertyLoader.cpp
      	xml/XMLPropertySaver.cpp
      	xml/XMLProximitySetupLoader.cpp
  	)

    set(FILES_HPP ${FILES_HPP}
        xml/XercesErrorHandler.hpp
      	xml/XercesUtils.hpp
      	xml/XMLBasisTypes.hpp
      	xml/XMLPathFormat.hpp
      	xml/XMLPathLoader.hpp
      	xml/XMLPathSaver.hpp
      	xml/XMLTrajectoryFormat.hpp
      	xml/XMLTrajectoryLoader.hpp
      	xml/XMLTrajectorySaver.hpp
      	xml/XMLPropertyFormat.hpp
      	xml/XMLPropertyLoader.hpp
      	xml/XMLPropertySaver.hpp
      	xml/XMLProximitySetupLoader.hpp
    )
ENDIF()

SOURCE_GROUP(loaders FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${LOADERS_FILES_HPP} DESTINATION "include/rw/loaders/")
