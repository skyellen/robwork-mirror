SET(FILES_CPP
	ConstraintModel.cpp
	ConstraintGenerator.cpp
	PointConstraint.cpp
	LineConstraint.cpp
	StablePoseConstraint.cpp
	BoxConstraint.cpp
	PlaneConstraint.cpp
	FixtureConstraint.cpp
)

SET(FILES_HPP
    ConstraintModel.hpp
    ConstraintGenerator.hpp
    PointConstraint.hpp
    LineConstraint.hpp
    StablePoseConstraint.hpp
	BoxConstraint.hpp
	PlaneConstraint.hpp
	FixtureConstraint.hpp
)

SOURCE_GROUP(constraints FILES ${FILES_CPP} ${FILES_HPP})

FILE(GLOB SRC_FILES_CPP ${FILES_CPP})
FILE(GLOB SRC_FILES_HPP ${FILES_HPP})

SET(ENV{RW_LIB_FILES_CPP} "$ENV{RW_LIB_FILES_CPP}${SRC_FILES_CPP};")
SET(ENV{RW_LIB_FILES_HPP} "$ENV{RW_LIB_FILES_HPP}${SRC_FILES_HPP};")

#INSTALL(FILES ${CONSTRAINTS_FILES_HPP} DESTINATION "include/rw/constraints")
