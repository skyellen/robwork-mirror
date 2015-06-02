SET(FILES_CPP
	${FILES_CPP}
	${CMAKE_CURRENT_SOURCE_DIR}/Optimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/StopCondition.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/OptimizerFactory.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/DownhillOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LineSearch.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoldenSectionLineSearch.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/DirectionSetOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TaxiCabOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/PowellOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GradientOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GradientDescentOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/BFGSOptimizer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SimulatedAnnealingOptimizer.cpp
	CACHE INTERNAL ""
)

SET(FILES_HPP
	${FILES_HPP}
	${CMAKE_CURRENT_SOURCE_DIR}/Optimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/StopCondition.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/OptimizerFactory.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/LineSearch.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/GoldenSectionLineSearch.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/DownhillOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/DirectionSetOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/TaxiCabOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/PowellOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/GradientOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/GradientDescentOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/BFGSOptimizer.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/SimulatedAnnealingOptimizer.hpp
	CACHE INTERNAL ""
)
